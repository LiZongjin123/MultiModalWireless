import yaml
from mountable import Mountable
from utils import Utils

class Cav(Mountable):

    def __init__(self, actor, index, sensors_config, root_dir_path):
        save_dir_path = root_dir_path + f"/cav{index}"
        super().__init__(index, sensors_config, actor, save_dir_path)

    def _attach_sensors(self):
        self._attach_sensor("camera")
        self._attach_sensor("lidar")
        self._attach_sensor("imu")

    def set_autopilot(self, is_enabled, *args, **kwargs):
        self._actor.set_autopilot(is_enabled, *args, **kwargs)

    def _camera_saving(self):
        for i in range(0, 4):
            image = self._sensor_queues[f"camera{i}"].get()
            image.save_to_disk(f"{self._save_dir_path}/{image.frame}_camera{i}.png")

    def save_data(self, vehicles, cavs):
        self._camera_saving()
        self._lidar_saving()
        self.__yaml_data_saving(vehicles, cavs)

    # TODO:增加rsu的数据
    def __yaml_data_saving(self, vehicles, cavs):
        all_camera_data = self.__generate_yaml_data_of_camera()

        cav_speed = self._actor.get_velocity()
        cav_transform = self._actor.get_transform()
        cav_location = cav_transform.location
        cav_rotation = cav_transform.rotation

        lidar_transform = self._sensors["lidar"].get_transform()
        lidar_location = lidar_transform.location
        lidar_rotation = lidar_transform.rotation

        imu_measurement = self._sensor_queues["imu"].get()
        frame_id = imu_measurement.frame
        accelerometer = imu_measurement.accelerometer
        gyroscope = imu_measurement.gyroscope
        compass = imu_measurement.compass

        all_vehicle_data_except_this_cav = {}
        for vehicle in vehicles:
            vehicle_data = self.__generate_yaml_data_of_vehicle(vehicle)
            all_vehicle_data_except_this_cav[vehicle.id] = vehicle_data

        for cav in cavs:
            if cav._index == self._index:
                continue
            vehicle_data = self.__generate_yaml_data_of_vehicle(cav._actor)
            all_vehicle_data_except_this_cav[cav._actor.id] = vehicle_data

        data = {
            "actor": f"cav_{self._index}",
            "frame": frame_id,
            "sensors": {
                "cameras": all_camera_data,
                "vehicle_speed": {
                    "speed": {
                        "x": cav_speed.x,
                        "y": cav_speed.y,
                        "z": cav_speed.z
                    }
                },
                "vehicle_pose": {
                    "location": {
                        "x": cav_location.x,
                        "y": cav_location.y,
                        "z": cav_location.z
                    },
                    "rotation": {
                        "pitch": cav_rotation.pitch,
                        "roll": cav_rotation.roll,
                        "yaw": cav_rotation.yaw
                    }
                },
                "lidar_pose": {
                    "location": {
                        "x": lidar_location.x,
                        "y": lidar_location.y,
                        "z": lidar_location.z
                    },
                    "rotation": {
                        "pitch": lidar_rotation.pitch,
                        "roll": lidar_rotation.roll,
                        "yaw": lidar_rotation.yaw
                    }
                },
                "imu_measurement": {
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
            },
            "vehicles": all_vehicle_data_except_this_cav
        }

        yaml_file_path = self._save_dir_path + f"/{frame_id}.yaml"
        with open(yaml_file_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    def __generate_yaml_data_of_vehicle(self, vehicle):
        vehicle_color_rgb = vehicle.attributes.get("color")
        if vehicle_color_rgb is None:
            vehicle_color_rgb = "None"

        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation

        vehicle_bounding_box = vehicle.bounding_box
        vehicle_center = vehicle_bounding_box.location
        vehicle_extent = vehicle_bounding_box.extent

        vehicle_speed_l2_norm = Utils.speed_l2_norm(vehicle.get_velocity())

        vehicle_data = {
            "bp_id": vehicle.type_id,
            "color": vehicle_color_rgb,
            "location": {
                "x": vehicle_location.x,
                "y": vehicle_location.y,
                "z": vehicle_location.z
            },
            "center": {
                "x": vehicle_center.x,
                "y": vehicle_center.y,
                "z": vehicle_center.z
            },
            "angle": {
                "pitch": vehicle_rotation.pitch,
                "roll": vehicle_rotation.roll,
                "yaw": vehicle_rotation.yaw
            },
            "extent": {
                "x": vehicle_extent.x,
                "y": vehicle_extent.y,
                "z": vehicle_extent.z
            },
            "speed": vehicle_speed_l2_norm
        }
        return vehicle_data

    def __generate_yaml_data_of_camera(self):
        all_camera_data = {}
        for i in range(0, 4):
            camera_name = f"camera{i}"
            camera_transform = self._sensors[camera_name].get_transform()
            camera_location = camera_transform.location
            camera_rotation = camera_transform.rotation
            camera_data = {
                "location": {
                    "x": camera_location.x,
                    "y": camera_location.y,
                    "z": camera_location.z
                },
                "rotation": {
                    "pitch": camera_rotation.pitch,
                    "roll": camera_rotation.roll,
                    "yaw": camera_rotation.yaw
                }
            }
            all_camera_data[camera_name] = camera_data
        return all_camera_data
