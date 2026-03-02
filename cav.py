from mountable import Mountable

class Cav(Mountable):

    def __init__(self, actor, index, sensors_config, root_dir_path):
        save_dir_path = root_dir_path + f"/cav_{index}"
        super().__init__(index, sensors_config, actor, save_dir_path, f"cav_{index}")

    def _attach_sensors(self):
        self._attach_sensor("camera0")
        self._attach_sensor("camera1")
        self._attach_sensor("camera2")
        self._attach_sensor("camera3")
        self._attach_sensor("lidar")
        self._attach_sensor("imu")

    def set_autopilot(self, is_enabled, *args, **kwargs):
        self._actor.set_autopilot(is_enabled, *args, **kwargs)

    def _camera_saving(self):
        for i in range(0, 4):
            image = self._sensor_queues[f"camera{i}"].get()
            image.save_to_disk(f"{self._save_dir_path}/{image.frame}_camera{i}.png")

    def save_data(self, actors):
        self._camera_saving()
        self._lidar_saving()
        self._yaml_data_saving(actors)

    def _generate_yaml_data_of_cav_speed(self):
        cav_speed = self._actor.get_velocity()
        cav_speed_yaml_data = {
            "speed": {
                "x": cav_speed.x,
                "y": cav_speed.y,
                "z": cav_speed.z
            }
        }
        return cav_speed_yaml_data

    def _generate_yaml_data_of_imu_measurement(self):
        imu_measurement = self._sensor_queues["imu"].get()
        accelerometer = imu_measurement.accelerometer
        gyroscope = imu_measurement.gyroscope
        compass = imu_measurement.compass
        yaml_data_of_imu = {
            "accelerometer": {
                "x": accelerometer.x,
                "y": accelerometer.y,
                "z": accelerometer.z
            },
            "gyroscope": {
                "x": gyroscope.x,
                "y": gyroscope.y,
                "z": gyroscope.z
            },
            "compass": compass
        }
        return yaml_data_of_imu

    def _generate_yaml_data_of_sensors(self):
        yaml_data_of_sensor_poses = self._generate_yaml_data_of_all_sensor_poses()
        yaml_data_of_self_actor_pose = self._generate_yaml_data_of_self_actor_pose()
        yaml_data_of_self_actor_speed = self._generate_yaml_data_of_cav_speed()
        yaml_data_of_imu_measurement = self._generate_yaml_data_of_imu_measurement()

        yaml_data_of_sensors = yaml_data_of_sensor_poses
        yaml_data_of_sensors["cav_pose"] = yaml_data_of_self_actor_pose
        yaml_data_of_sensors["cav_speed"] = yaml_data_of_self_actor_speed
        yaml_data_of_sensors["imu_measurement"] = yaml_data_of_imu_measurement

        return yaml_data_of_sensors
