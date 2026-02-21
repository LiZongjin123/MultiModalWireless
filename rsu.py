import json

import carla
from mountable import Mountable

class Rsu(Mountable):

    def __init__(self, actor, index, sensors_config, root_dir_path):
        save_dir_path = root_dir_path + f"/rsu_{index}"
        super().__init__(index, sensors_config, actor, save_dir_path, f"rsu_{index}")

    def _attach_sensors(self):
        self._attach_sensor("camera")
        self._attach_sensor("depth_camera")
        self._attach_sensor("lidar")
        self._attach_sensor("radar")

    def save_data(self, actors):
        self._camera_saving()
        self._depth_camera_saving()
        self._lidar_saving()
        self._radar_saving()
        self._yaml_data_saving(actors)

    def _camera_saving(self):
        image = self._sensor_queues["camera"].get()
        image.save_to_disk(f"{self._save_dir_path}/{image.frame}_camera.png")

    def _depth_camera_saving(self):
        image = self._sensor_queues["depth_camera"].get()
        image.save_to_disk(f"{self._save_dir_path}/{image.frame}_depth_camera.png", carla.ColorConverter.LogarithmicDepth)

    def _radar_saving(self):
        radar_measurement = self._sensor_queues["radar"].get()
        json_data_of_radar_measurement = []
        for detection in radar_measurement:
            json_data_of_detection = {
                "velocity": detection.velocity,
                "azimuth": detection.azimuth,
                "altitude": detection.altitude,
                "depth": detection.depth
            }
            json_data_of_radar_measurement.append(json_data_of_detection)

        frame_id = radar_measurement.frame
        with open(f"{self._save_dir_path}/{frame_id}.json", 'w', encoding='utf-8') as f:
            json.dump(json_data_of_radar_measurement, f, ensure_ascii=False, indent=4)

    def _generate_yaml_data_of_sensors(self):
        yaml_data_of_sensor_poses = self._generate_yaml_data_of_all_sensor_poses()
        yaml_data_of_self_actor_pose = self._generate_yaml_data_of_self_actor_pose()

        yaml_data_of_sensors = yaml_data_of_sensor_poses
        yaml_data_of_sensors["rsu_pose"] = yaml_data_of_self_actor_pose

        return yaml_data_of_sensors
