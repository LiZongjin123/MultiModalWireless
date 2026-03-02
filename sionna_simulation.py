import os
import sionna
import tensorflow as tf
import numpy as np
import yaml
import drjit as dr
import mitsuba as mi

class SionnaSimulation:
    def __init__(self):
        self.__config = None
        self.__gpu_num = None
        self.__output_dir_path = None
        self.__carla_output_dir_path = None
        self.__blender_output_dir_path = None
        self.__sionna_output_dir_path = None
        self.__sionna_config = None
        self.__scenario_config = None

    def init(self, config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        self.__config = config
        self.__gpu_num = self.__config["sionna"]["gpu_num"]
        self.__output_dir_path = self.__config["output_path"]
        self.__carla_output_dir_path = os.path.join(self.__output_dir_path, "carla_output")
        self.__blender_output_dir_path = os.path.join(self.__output_dir_path, "blender_output")
        self.__sionna_output_dir_path = os.path.join(self.__output_dir_path, "sionna_output")
        self.__scenario_config = config["scenario"]

        os.environ["CUDA_VISIBLE_DEVICES"] = f"{self.__gpu_num}"
        gpus = tf.config.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(gpus[0], True)
        self.__sionna_config = {"tx_row": 1, "tx_col": 16, "rx_row": 1, "rx_col": 16, "fc": 5e9}

        cav_num = len(self.__scenario_config["desired_cav_ranks"])
        os.makedirs(self.__sionna_output_dir_path, exist_ok=True)
        for i in range(0, cav_num):
            cav_save_dir = os.path.join(self.__sionna_output_dir_path, f"cav_{i}")
            os.makedirs(cav_save_dir, exist_ok=True)

    def generate_channel_data(self):
        mitsuba_scene_files = sorted([file for file in os.listdir(self.__blender_output_dir_path) if file.endswith(".xml")])

        for mitsuba_scene_file_name in mitsuba_scene_files:
            frame_id = os.path.splitext(mitsuba_scene_file_name)[0]
            mitsuba_scene_file_path = os.path.join(self.__blender_output_dir_path, mitsuba_scene_file_name)
            sionna_scene = sionna.rt.load_scene(mitsuba_scene_file_path)

            self.__set_planar_array(sionna_scene)

            rsu_yaml_file_dir = os.path.join(self.__carla_output_dir_path, "rsu_0")
            rsu_yaml_file = os.path.join(rsu_yaml_file_dir, f"{frame_id}.yaml")
            rsu_planar_array_pose = SionnaSimulation.__load_actor_planar_array_pose(rsu_yaml_file)

            tx = sionna.rt.Transmitter(name="tx", position=rsu_planar_array_pose["position"],
                                       orientation=rsu_planar_array_pose["orientation"])
            sionna_scene.add(tx)
            sionna_scene.frequency = self.__sionna_config["fc"]
            sionna_scene.synthetic_array = True

            cav_num = len(self.__scenario_config["desired_cav_ranks"])
            for i in range(0, cav_num):
                cav_yaml_file_dir = os.path.join(self.__carla_output_dir_path, f"cav_{i}")
                cav_yaml_file = os.path.join(cav_yaml_file_dir, f"{frame_id}.yaml")
                cav_planar_array_pose = SionnaSimulation.__load_actor_planar_array_pose(cav_yaml_file)
                rx = sionna.rt.Receiver(name=f"rx_{i}", position=cav_planar_array_pose["position"],
                                        orientation=cav_planar_array_pose["orientation"])
                sionna_scene.add(rx)

                solver = sionna.rt.PathSolver()
                paths = solver(sionna_scene, max_depth=1)
                a, tau = paths.cir()
                theta_t, phi_t = SionnaSimulation.__aod_transform(paths.theta_t, paths.phi_t,
                                                                  rsu_planar_array_pose["orientation"])
                theta_r, phi_r = SionnaSimulation.__aod_transform(paths.theta_r, paths.phi_r,
                                                                  cav_planar_array_pose["orientation"])

                paths_data = {
                    "a": a,
                    "tau": tau,
                    "theta_t": theta_t,
                    "phi_t": phi_t,
                    "theta_r": theta_r,
                    "phi_r": phi_r,
                    "global_theta_t": paths.theta_t,
                    "global_phi_t": paths.phi_t,
                    "global_theta_r": paths.theta_r,
                    "global_phi_r": paths.phi_r
                }

                paths_data_save_dir = os.path.join(self.__sionna_output_dir_path, f"cav_{i}")
                SionnaSimulation.__paths_data_save(paths_data, paths_data_save_dir, frame_id)

                sionna_scene.remove(rx.name)
            sionna_scene.remove(tx.name)
            print(f"frame_id: {frame_id}")

    @staticmethod
    def __aod_transform(global_theta, global_phi, planar_array_orientation):
        global_theta = dr.ravel(global_theta)
        global_phi = dr.ravel(global_phi)
        path_global_orientation = sionna.rt.r_hat(global_theta, global_phi)

        planar_array_orientation = mi.Point3f(planar_array_orientation)
        rotation_matrix = sionna.rt.rotation_matrix(planar_array_orientation)
        path_local_orientation = rotation_matrix @ path_global_orientation

        local_theta, local_phi = sionna.rt.theta_phi_from_unit_vec(path_local_orientation)
        return local_theta, local_phi

    def __set_planar_array(self, sionna_scene):
        sionna_scene.tx_array = sionna.rt.PlanarArray(num_rows=self.__sionna_config["tx_row"],
                                                      num_cols=self.__sionna_config["tx_col"],
                                                      vertical_spacing=0.5,
                                                      horizontal_spacing=0.5,
                                                      pattern="dipole",
                                                      polarization="V")

        sionna_scene.rx_array = sionna.rt.PlanarArray(num_rows=self.__sionna_config["rx_row"],
                                                      num_cols=self.__sionna_config["rx_col"],
                                                      vertical_spacing=0.5,
                                                      horizontal_spacing=0.5,
                                                      pattern="dipole",
                                                      polarization="V")

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
    def __load_actor_planar_array_pose(actor_yaml_file_path):
        with open(actor_yaml_file_path, 'r', encoding="utf-8") as f:
            yaml_file_data = yaml.safe_load(f)
        carla_lidar_location = yaml_file_data["sensors"]["lidar"]["location"]
        carla_lidar_rotation = yaml_file_data["sensors"]["lidar"]["rotation"]
        sionna_lidar_location = [carla_lidar_location["x"],
                                 -carla_lidar_location["y"],
                                 carla_lidar_location["z"]]
        sionna_lidar_rotation = [-carla_lidar_rotation["yaw"] * np.pi / 180,
                                 carla_lidar_rotation["pitch"] * np.pi / 180,
                                 carla_lidar_rotation["roll"] * np.pi / 180]
        planar_array_pose = {"position": sionna_lidar_location,
                             "orientation": sionna_lidar_rotation}
        return planar_array_pose

    @staticmethod
    def __paths_data_save(paths_data, paths_data_save_dir, frame_id):
        paths_data = {k: (v.numpy() if hasattr(v, 'numpy') else v) for k, v in paths_data.items()}
        paths_file = os.path.join(paths_data_save_dir, f"{frame_id}_paths.npz") # Added cav_id_str
        np.savez_compressed(paths_file, **paths_data)




