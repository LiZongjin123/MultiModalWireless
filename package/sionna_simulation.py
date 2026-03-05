import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import sionna
import tensorflow as tf
import numpy as np
import yaml
import mitsuba as mi

class SionnaSimulation:

    def __init__(self):
        self.__config = None
        self.__sionna_simulation_config = None
        self.__output_dir_path = None
        self.__carla_output_dir_path = None
        self.__blender_output_dir_path = None
        self.__sionna_output_dir_path = None
        self.__scenario_config = None
        self.__txs = []
        self.__rxs = []

    def init(self, config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        self.__config = config
        self.__sionna_simulation_config = config["sionna_simulation"]
        self.__output_dir_path = self.__config["output_path"]
        self.__carla_output_dir_path = os.path.join(self.__output_dir_path, "carla_output")
        self.__blender_output_dir_path = os.path.join(self.__output_dir_path, "blender_output")
        self.__sionna_output_dir_path = os.path.join(self.__output_dir_path, "sionna_output")
        self.__scenario_config = config["scenario"]

    def set_gpu(self):
        gpu_num = self.__sionna_simulation_config["gpu_num"]
        os.environ["CUDA_VISIBLE_DEVICES"] = f"{gpu_num}"
        gpus = tf.config.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(gpus[gpu_num], True)

    def creat_save_dir(self):
        os.makedirs(self.__sionna_output_dir_path, exist_ok=True)

    def sionna_scene_preview(self, frame_id, is_show_paths, is_show_orientations):
        mitsuba_scene_file_name = f"{frame_id}.xml"

        frame_id = os.path.splitext(mitsuba_scene_file_name)[0]
        mitsuba_scene_file_path = os.path.join(self.__blender_output_dir_path, mitsuba_scene_file_name)
        sionna_scene = sionna.rt.load_scene(mitsuba_scene_file_path)

        self.__config_planar_array(sionna_scene)
        sionna_scene.frequency = self.__sionna_simulation_config["fc"]
        self.__generate_planar_array(frame_id, sionna_scene)
        self.__set_actors_speed(frame_id, sionna_scene)

        if is_show_paths:
            paths = self.__get_paths(sionna_scene)
        else:
            paths = None

        sionna_scene.preview(paths=paths, show_orientations=is_show_orientations)

        self.__clear_planar_array(sionna_scene)

    def generate_channel_data(self):
        mitsuba_scene_files = sorted([file for file in os.listdir(self.__blender_output_dir_path) if file.endswith(".xml")])

        for mitsuba_scene_file_name in mitsuba_scene_files:
            frame_id = os.path.splitext(mitsuba_scene_file_name)[0]
            mitsuba_scene_file_path = os.path.join(self.__blender_output_dir_path, mitsuba_scene_file_name)
            sionna_scene = sionna.rt.load_scene(mitsuba_scene_file_path)

            self.__config_planar_array(sionna_scene)
            sionna_scene.frequency = self.__sionna_simulation_config["fc"]
            self.__set_actors_speed(frame_id, sionna_scene)
            self.__generate_planar_array(frame_id, sionna_scene)

            paths = self.__get_paths(sionna_scene)
            paths_data = self.__get_paths_data(paths)
            SionnaSimulation.__paths_data_save(paths_data, self.__sionna_output_dir_path, frame_id)

            self.__clear_planar_array(sionna_scene)
            print(f"frame_id: {frame_id}")

    def get_npz_file(self, frame_id):
        npz_file_path = os.path.join(self.__sionna_output_dir_path, f"{frame_id}.npz")
        npz_file = np.load(npz_file_path)
        print("all keys: ", npz_file.files)
        print()
        return npz_file

    def __get_paths_data(self, paths):
        sampling_frequency = self.__config["simulation"]["frame_rate"]

        cir_a, cir_tau = paths.cir(sampling_frequency=sampling_frequency,
                                   num_time_steps=self.__sionna_simulation_config["cir"]["num_time_steps"])

        ofdm_frequencies = self.__get_ofdm_frequencies()
        cfr = paths.cfr(ofdm_frequencies,
                        sampling_frequency=sampling_frequency,
                        num_time_steps=self.__sionna_simulation_config["cfr"]["num_time_steps"])

        paths_data = {
            "cir_a": cir_a,
            "cir_tau": cir_tau,
            "cfr": cfr,
            "doppler": paths.doppler,
            "theta_t": paths.theta_t,
            "phi_t": paths.phi_t,
            "theta_r": paths.theta_r,
            "phi_r": paths.phi_r
        }
        return paths_data

    def __get_paths(self, sionna_scene):
        paths_solver_config = self.__sionna_simulation_config["paths_solver"]
        solver = sionna.rt.PathSolver()
        paths = solver(sionna_scene,
                       max_depth=paths_solver_config["max_depth"],
                       max_num_paths_per_src=paths_solver_config["max_num_paths_per_src"],
                       samples_per_src=paths_solver_config["samples_per_src"],
                       synthetic_array=paths_solver_config["synthetic_array"])
        return paths

    def __clear_planar_array(self, sionna_scene):
        for tx in self.__txs:
            sionna_scene.remove(tx.name)
        self.__txs = []
        for rx in self.__rxs:
            sionna_scene.remove(rx.name)
        self.__rxs = []

    def __generate_planar_array(self, frame_id: str, sionna_scene):
        rsu_num = len(self.__scenario_config["rsu_transform"])
        for i in range(0, rsu_num):
            rsu_yaml_file_dir = os.path.join(self.__carla_output_dir_path, f"rsu_{i}")
            rsu_yaml_file = os.path.join(rsu_yaml_file_dir, f"{frame_id}.yaml")
            rsu_planar_array_pose_and_velocity = SionnaSimulation.__read_actor_planar_array_pose_and_velocity(
                rsu_yaml_file)

            tx = sionna.rt.Transmitter(name=f"tx_{i}", position=rsu_planar_array_pose_and_velocity["position"],
                                       orientation=rsu_planar_array_pose_and_velocity["orientation"],
                                       velocity=rsu_planar_array_pose_and_velocity["velocity"])
            sionna_scene.add(tx)
            self.__txs.append(tx)

        cav_num = len(self.__scenario_config["desired_cav_ranks"])
        for i in range(0, cav_num):
            cav_yaml_file_dir = os.path.join(self.__carla_output_dir_path, f"cav_{i}")
            cav_yaml_file = os.path.join(cav_yaml_file_dir, f"{frame_id}.yaml")
            cav_planar_array_pose_and_velocity = SionnaSimulation.__read_actor_planar_array_pose_and_velocity(
                cav_yaml_file)

            rx = sionna.rt.Receiver(name=f"rx_{i}", position=cav_planar_array_pose_and_velocity["position"],
                                    orientation=cav_planar_array_pose_and_velocity["orientation"],
                                    velocity=cav_planar_array_pose_and_velocity["velocity"])
            sionna_scene.add(rx)
            self.__rxs.append(rx)

    def __set_actors_speed(self, frame_id: str, sionna_scene):
        scene_yaml_file_dir = os.path.join(self.__carla_output_dir_path, "scene")
        scene_yaml_file = os.path.join(scene_yaml_file_dir, f"{frame_id}.yaml")
        actor_names = [object_name for object_name in sionna_scene.objects if object_name.startswith("actor_")]

        for actor_name in actor_names:
            actor_id = int(actor_name.split("_")[1])
            actor_speed = self.__read_actor_speed(scene_yaml_file, actor_id)
            sionna_scene.objects[actor_name].velocity = mi.Vector3f(actor_speed)

    def __config_planar_array(self, sionna_scene):
        planar_array_tx_config = self.__sionna_simulation_config["planar_array_tx"]
        sionna_scene.tx_array = sionna.rt.PlanarArray(num_rows=planar_array_tx_config["num_rows"],
                                                      num_cols=planar_array_tx_config["num_cols"],
                                                      vertical_spacing=planar_array_tx_config["vertical_spacing"],
                                                      horizontal_spacing=planar_array_tx_config["horizontal_spacing"],
                                                      pattern=planar_array_tx_config["pattern"],
                                                      polarization=planar_array_tx_config["polarization"])

        planar_array_rx_config = self.__sionna_simulation_config["planar_array_rx"]
        sionna_scene.rx_array = sionna.rt.PlanarArray(num_rows=planar_array_rx_config["num_rows"],
                                                      num_cols=planar_array_rx_config["num_cols"],
                                                      vertical_spacing=planar_array_rx_config["vertical_spacing"],
                                                      horizontal_spacing=planar_array_rx_config["horizontal_spacing"],
                                                      pattern=planar_array_rx_config["pattern"],
                                                      polarization=planar_array_rx_config["polarization"])

    def __add_radio_materio(self, sionna_scene):
        custom_itu_wet_ground_28 = sionna.rt.RadioMaterial("itu_wet_ground_28",
                                                    relative_permittivity=3,
                                                    conductivity=2.5,
                                                    scattering_coefficient=0.0,
                                                    xpd_coefficient=0.0,
                                                    scattering_pattern=sionna.rt.LambertianPattern(),
                                                    frequency_update_callback=None)
        sionna_scene.add(custom_itu_wet_ground_28)

        custom_itu_medium_dry_ground_28 = sionna.rt.RadioMaterial("itu_medium_dry_ground_28",
                                                           relative_permittivity=3,
                                                           conductivity=0.4,
                                                           scattering_coefficient=0.0,
                                                           xpd_coefficient=0.0,
                                                           scattering_pattern=sionna.rt.LambertianPattern())
        sionna_scene.add(custom_itu_medium_dry_ground_28)

        custom_itu_very_dry_ground_28 = sionna.rt.RadioMaterial("itu_very_dry_ground_28",
                                                         relative_permittivity=2.5,
                                                         conductivity=0.03,
                                                         scattering_coefficient=0.0,
                                                         xpd_coefficient=0.0,
                                                         scattering_pattern=sionna.rt.LambertianPattern())
        sionna_scene.add(custom_itu_very_dry_ground_28)

    @staticmethod
    def __read_actor_planar_array_pose_and_velocity(actor_yaml_file_path):
        with open(actor_yaml_file_path, 'r', encoding="utf-8") as f:
            yaml_file_data = yaml.safe_load(f)

        carla_lidar_location = yaml_file_data["sensors"]["lidar"]["location"]
        carla_lidar_rotation = yaml_file_data["sensors"]["lidar"]["rotation"]
        if "cav" in yaml_file_data["sensors"]:
            carla_actor_velocity = yaml_file_data["sensors"]["cav"]["speed"]
        elif "rsu" in yaml_file_data["sensors"]:
            carla_actor_velocity = yaml_file_data["sensors"]["rsu"]["speed"]

        sionna_lidar_location = [carla_lidar_location["x"],
                                 -carla_lidar_location["y"],
                                 carla_lidar_location["z"]]
        sionna_lidar_rotation = [-carla_lidar_rotation["yaw"] * np.pi / 180,
                                 carla_lidar_rotation["pitch"] * np.pi / 180,
                                 carla_lidar_rotation["roll"] * np.pi / 180]
        sionna_actor_velocity = [carla_actor_velocity["x"],
                                 -carla_actor_velocity["y"],
                                 carla_actor_velocity["z"]]
        planar_array_pose = {"position": sionna_lidar_location,
                             "orientation": sionna_lidar_rotation,
                             "velocity": sionna_actor_velocity}
        return planar_array_pose

    @staticmethod
    def __paths_data_save(paths_data, paths_data_save_dir, frame_id):
        paths_data = {k: (v.numpy() if hasattr(v, 'numpy') else v) for k, v in paths_data.items()}
        paths_file = os.path.join(paths_data_save_dir, f"{frame_id}.npz") # Added cav_id_str
        np.savez(paths_file, **paths_data)

    def __get_ofdm_frequencies(self):
        num_subcarriers = self.__sionna_simulation_config["num_subcarriers"]
        subcarrier_spacing = self.__sionna_simulation_config["subcarrier_spacing"]
        fc = self.__sionna_simulation_config["fc"]

        subcarrier_frequencies = sionna.rt.subcarrier_frequencies(num_subcarriers, subcarrier_spacing)
        return fc + subcarrier_frequencies

    @staticmethod
    def __read_actor_speed(scene_yaml_file, actor_id):
        with open(scene_yaml_file, 'r', encoding="utf-8") as f:
            scene_data = yaml.safe_load(f)
        carla_actor_speed = scene_data["actors"][actor_id]["speed"]
        sionna_actor_speed = [carla_actor_speed["x"],
                              -carla_actor_speed["y"],
                              carla_actor_speed["z"]]
        return sionna_actor_speed



