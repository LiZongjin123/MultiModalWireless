import os
import sionna
import tensorflow as tf
import numpy as np
import yaml
from sionna.rt import safe_atan2

class SionnaScript:
    def __init__(self, gpu_num, carla_output_dir_path, blender_output_dir_path, scenario_config,
                 sionna_output_dir_path):
        self.__gpu_num = gpu_num
        self.__carla_output_dir_path = carla_output_dir_path
        self.__blender_output_dir_path = blender_output_dir_path
        self.__sionna_config = None

        self.__scenario_config = scenario_config
        self.__sionna_output_dir_path = sionna_output_dir_path


    def init(self):
        os.environ["CUDA_VISIBLE_DEVICES"] = f"{self.__gpu_num}"
        gpus = tf.config.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(gpus[0], True)
        self.__sionna_config = {"tx_row": 1, "tx_col": 16, "rx_row": 1, "rx_col": 16, "fc": 28e9}

        cav_num = len(self.__scenario_config["actors"]["desired_cav_ranks"])
        os.makedirs(self.__sionna_output_dir_path, exist_ok=True)
        for i in range(0, cav_num):
            cav_save_dir = os.path.join(self.__sionna_output_dir_path, f"cav_{i}")
            os.makedirs(cav_save_dir, exist_ok=True)

    def generate_channel_data(self):
        mitsuba_scene_files = sorted([file for file in os.listdir(self.__blender_output_dir_path) if file.endswith(".xml")])

        for mitsuba_scene_file in mitsuba_scene_files:
            frame_id = os.path.splitext(mitsuba_scene_file)[0]
            sionna_scene = sionna.rt.load_scene(mitsuba_scene_file)

            self.__add_radio_materio(sionna_scene)

            self.__set_planar_array(sionna_scene)

            rsu_yaml_file_dir = os.path.join(self.__carla_output_dir_path, "rsu_0")
            rsu_yaml_file = os.path.join(rsu_yaml_file_dir, f"{frame_id}.yaml")
            rsu_planar_array_pose = SionnaScript.__load_actor_planar_array_pose(rsu_yaml_file)

            tx = sionna.rt.Transmitter(name="tx", position=rsu_planar_array_pose["position"],
                                       orientation=rsu_planar_array_pose["orientation"])
            sionna_scene.add(tx)
            sionna_scene.frequency = self.__sionna_config["fc"]
            sionna_scene.synthetic_array = True

            cav_num = len(self.__scenario_config["actors"]["desired_cav_ranks"])
            for i in range(0, cav_num):
                cav_yaml_file_dir = os.path.join(self.__carla_output_dir_path, f"cav_{i}")
                cav_yaml_file = os.path.join(cav_yaml_file_dir, f"{frame_id}.yaml")
                cav_planar_array_pose = SionnaScript.__load_actor_planar_array_pose(cav_yaml_file)
                rx = sionna.rt.Receiver(name=f"rx_{i}", position=cav_planar_array_pose["position"],
                                        orientation=cav_planar_array_pose["orientation"])
                sionna_scene.add(rx)

                paths = sionna_scene.compute_paths(max_depth=1, num_samples=1e6)
                a, tau = paths.cir()
                theta_t, phi_t = SionnaScript.__aod_transform(paths.theta_t, paths.phi_t,
                                                              rsu_planar_array_pose["orientation"])
                theta_r, phi_r = SionnaScript.__aod_transform(paths.theta_r, paths.phi_r,
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
                SionnaScript.__paths_data_save(paths_data, paths_data_save_dir, frame_id)

                sionna_scene.remove(rx.name)
            sionna_scene.remove(tx.name)
            print(f"frame_id: {frame_id}")


    @staticmethod
    def __aod_transform(global_theta, global_phi, planar_array_orientation):
        path_global_orientation = sionna.rt.r_hat(global_theta, global_phi)

        planar_array_orientation_tensor = tf.convert_to_tensor(planar_array_orientation, dtype=tf.float32)
        rotation_matrix = tf.transpose(sionna.rt.rotation_matrix(planar_array_orientation_tensor))
        rotation_matrix = sionna.rt.expand_to_rank(rotation_matrix, tf.rank(path_global_orientation) + 1, 0)
        path_local_orientation = tf.linalg.matvec(rotation_matrix, path_global_orientation)

        local_theta, local_phi = sionna.rt.theta_phi_from_unit_vec(path_local_orientation)
        return local_theta, local_phi

    def __set_planar_array(self, sionna_scene):
        sionna_scene.tx_array = sionna.rt.PlanarArray(num_rows=self.__sionna_config["tx_rows"],
                                                      num_cols=self.__sionna_config["tx_cols"],
                                                      vertical_spacing=0.5,
                                                      horizontal_spacing=0.5,
                                                      pattern="dipole",
                                                      polarization="V")

        sionna_scene.rx_array = sionna.rt.PlanarArray(num_rows=self.__sionna_config["rx_rows"],
                                                      num_cols=self.__sionna_config["rx_cols"],
                                                      vertical_spacing=0.5,
                                                      horizontal_spacing=0.5,
                                                      pattern="dipole",
                                                      polarization="V")

    def __add_radio_materio(self, sionna_scene):
        itu_wet_ground_28 = sionna.rt.RadioMaterial("itu_wet_ground_28",
                                                    relative_permittivity=3,
                                                    conductivity=2.5,
                                                    scattering_coefficient=0.0,
                                                    xpd_coefficient=0.0,
                                                    scattering_pattern=sionna.rt.LambertianPattern(),
                                                    frequency_update_callback=None)
        sionna_scene.add(itu_wet_ground_28)

        itu_medium_dry_ground_28 = sionna.rt.RadioMaterial("itu_medium_dry_ground_28",
                                                           relative_permittivity=3,
                                                           conductivity=0.4,
                                                           scattering_coefficient=0.0,
                                                           xpd_coefficient=0.0,
                                                           scattering_pattern=sionna.rt.LambertianPattern())
        sionna_scene.add(itu_medium_dry_ground_28)

        itu_very_dry_ground_28 = sionna.rt.RadioMaterial("itu_very_dry_ground_28",
                                                         relative_permittivity=2.5,
                                                         conductivity=0.03,
                                                         scattering_coefficient=0.0,
                                                         xpd_coefficient=0.0,
                                                         scattering_pattern=sionna.rt.LambertianPattern())
        sionna_scene.add(itu_very_dry_ground_28)

    @staticmethod
    def __load_actor_planar_array_pose(actor_yaml_file_path):
        with open(actor_yaml_file_path, 'r', encoding="utf-8") as f:
            yaml_file_data = yaml.safe_load(f)
        carla_lidar_location = yaml_file_data["sensor"]["lidar"]["location"]
        carla_lidar_rotation = yaml_file_data["sensor"]["lidar"]["rotation"]
        sionna_lidar_location = [carla_lidar_location["x"],
                                 -carla_lidar_rotation["y"],
                                 carla_lidar_location["z"]]
        sionna_lidar_rotation = [-carla_lidar_rotation["yaw"] * np.pi / 180,
                                 carla_lidar_rotation["pitch"] * np.pi / 180,
                                 carla_lidar_rotation["roll"] * np.pi / 180]
        planar_array_pose = {"position": sionna_lidar_location,
                             "orientation": sionna_lidar_rotation}
        return planar_array_pose
        # actor_name = yaml_file_data["actor"]
        # if actor_name.startswith("rsu"):
        # elif actor_name.startswith("cav"):
        #     carla_cav_speed = yaml_file_data["sensors"]["cav_speed"]["speed"]
        #     sionna_cav_speed = [carla_cav_speed["x"], -carla_cav_speed["y"], carla_cav_speed["z"]]

    @staticmethod
    def __paths_data_save(paths_data, paths_data_save_dir, frame_id):
        paths_data = {k: (v.numpy() if hasattr(v, 'numpy') else v) for k, v in paths_data.items()}
        paths_file = os.path.join(paths_data_save_dir, f"{frame_id}_paths.npz") # Added cav_id_str
        np.savez_compressed(paths_file, **paths_data)


config_path = "./config.yaml"
with open(config_path, "r") as f:
    config = yaml.safe_load(f)

scenario_name = config["scenario"]
scenario_config = config["scenarios"][scenario_name]
sionna_script = SionnaScript(0,
                             "./Town03_gastation_seed40",
                             "./blender_output",
                             scenario_config,
                             "./sionna_output")
sionna_script.init()
sionna_script.generate_channel_data()


