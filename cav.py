import math
import os
import struct
import carla
import queue
import yaml


class Cav:
    def __init__(self, vehicle, index, sensors_config, root_dir_path):
        self.__index = index
        self.__vehicle = vehicle
        self.__blueprint_library = vehicle.get_world().get_blueprint_library()
        self.__world = vehicle.get_world()
        self.__sensors = {}
        self.__save_dir_path = root_dir_path + f"/cav{index}"
        self.__sensors_config = sensors_config
        self.__image_queues = [queue.Queue() for _ in range(0, 4)]
        self.__sensor_queues = {}

    def init(self, need_cav_attach_sensors):
        os.makedirs(self.__save_dir_path, exist_ok=True)
        if need_cav_attach_sensors:
            self.__attach_sensors()

    def destroy(self):
        for sensor in self.__sensors.values():
            sensor.stop()
            sensor.destroy()
        self.__vehicle.destroy()

    def __attach_camera(self, index):
        camera_config = self.__sensors_config["cav"]["camera"]
        carla_location = carla.Location(x=camera_config["transforms"][f"camera{index}"]["location"]["x"],
                                        y=camera_config["transforms"][f"camera{index}"]["location"]["y"],
                                        z=camera_config["transforms"][f"camera{index}"]["location"]["z"])
        carla_rotation = carla.Rotation(yaw=camera_config["transforms"][f"camera{index}"]["rotation"]["yaw"])
        camera_transform = carla.Transform(carla_location, carla_rotation)

        camera_blueprint = self.__blueprint_library.find(camera_config["blueprint"])
        camera_attributes = camera_config["attributes"]
        for key, value in camera_attributes.items():
            camera_blueprint.set_attribute(key, str(value))

        camera = self.__world.spawn_actor(camera_blueprint, camera_transform, attach_to=self.__vehicle)
        camera.listen(self.__image_queues[index].put)
        self.__sensors[f"camera{index}"] = camera

    def __save_image(self):
        for i in range(0, 4):
            image = self.__image_queues[i].get()
            image.save_to_disk(f"{self.__save_dir_path}/{image.frame}_camera{i}.png")

    def warmup(self):
        for i in range(0, 4):
            image = self.__image_queues[i].get()
        for sensor_queue in self.__sensor_queues.values():
            data = sensor_queue.get()

    def __attach_cameras(self):
        for index in range(0, 4):
            self.__attach_camera(index)

    def set_autopilot(self, is_enabled, *args, **kwargs):
        self.__vehicle.set_autopilot(is_enabled, *args, **kwargs)

    def __attach_sensors(self):
        self.__attach_cameras()
        self.__attach_lidar()
        self.__attach_imu()

    def __attach_lidar(self):
        lidar_config = self.__sensors_config["cav"]["lidar"]
        lidar_blueprint = self.__blueprint_library.find(lidar_config["blueprint"])

        lidar_location = carla.Location(x=lidar_config["transform"]["location"]["x"],
                                        z=lidar_config["transform"]["location"]["z"])
        lidar_transform = carla.Transform(lidar_location)
        lidar_attributes = lidar_config["attributes"]
        for key, value in lidar_attributes.items():
            lidar_blueprint.set_attribute(key, str(value))

        lidar = self.__world.spawn_actor(lidar_blueprint, lidar_transform, attach_to=self.__vehicle)
        self.__sensor_queues["lidar"] = queue.Queue()
        lidar.listen(self.__sensor_queues["lidar"].put)
        self.__sensors["lidar"] = lidar

    def __attach_imu(self):
        imu_config = self.__sensors_config["cav"]["imu"]
        imu_blueprint = self.__blueprint_library.find(imu_config["blueprint"])

        imu_location = carla.Location(x=imu_config["transform"]["location"]["x"],
                                      y=imu_config["transform"]["location"]["y"],
                                      z=imu_config["transform"]["location"]["z"])

        imu_transform = carla.Transform(imu_location)
        imu_attributes = imu_config["attributes"]
        for key, value in imu_attributes.items():
            imu_blueprint.set_attribute(key, str(value))

        imu = self.__world.spawn_actor(imu_blueprint, imu_transform, attach_to=self.__vehicle)
        self.__sensor_queues["imu"] = queue.Queue()
        imu.listen(self.__sensor_queues["imu"].put)
        self.__sensors["imu"] = imu

    def __save_lidar_to_pcd(self):
        lidar_measurement = self.__sensor_queues["lidar"].get()
        point_format = 'ffff'
        point_size = struct.calcsize(point_format)
        points = []
        for i in range(0, len(lidar_measurement.raw_data), point_size):
            raw_point = lidar_measurement.raw_data[i:i + point_size]
            x, y, z, intensity = struct.unpack(point_format, raw_point)
            points.append(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.6f}")

        pcd_header = [
            '# .PCD v0.7 - Point Cloud Data file format',
            'VERSION 0.7',
            'FIELDS x y z intensity',
            'SIZE 4 4 4 4',
            'TYPE F F F F',
            'COUNT 1 1 1 1',
            f'WIDTH {len(points)}',
            'HEIGHT 1',
            'VIEWPOINT 0 0 0 1 0 0 0',
            f'POINTS {len(points)}',
            'DATA ascii'
        ]

        pcd_content = '\n'.join(pcd_header) + '\n' + '\n'.join(points)

        frame_id = lidar_measurement.frame
        pcd_file_path = self.__save_dir_path + f"/{frame_id}.pcd"

        with open(pcd_file_path, 'w', encoding='utf-8') as f:
            f.write(pcd_content)

    def save_data(self, vehicles, cavs):
        self.__save_image()
        self.__save_lidar_to_pcd()
        self.__save_numeric_data_as_yaml(vehicles, cavs)

    # TODO:增加rsu的数据
    def __save_numeric_data_as_yaml(self, vehicles, cavs):
        all_camera_data = self.__generate_camera_data_of_yaml()

        cav_speed = self.__vehicle.get_velocity()
        cav_transform = self.__vehicle.get_transform()
        cav_location = cav_transform.location
        cav_rotation = cav_transform.rotation

        lidar_transform = self.__sensors["lidar"].get_transform()
        lidar_location = lidar_transform.location
        lidar_rotation = lidar_transform.rotation

        imu_measurement = self.__sensor_queues["imu"].get()
        frame_id = imu_measurement.frame
        accelerometer = imu_measurement.accelerometer
        gyroscope = imu_measurement.gyroscope
        compass = imu_measurement.compass

        all_vehicle_data_except_this_cav = {}
        for vehicle in vehicles:
            vehicle_data = self.__generate_vehicle_data_of_yaml(vehicle)
            all_vehicle_data_except_this_cav[vehicle.id] = vehicle_data

        for cav in cavs:
            if cav.__index == self.__index:
                continue
            vehicle_data = self.__generate_vehicle_data_of_yaml(cav.__vehicle)
            all_vehicle_data_except_this_cav[cav.__vehicle.id] = vehicle_data

        data = {
            "actor": f"cav_{self.__index}",
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

        yaml_file_path = self.__save_dir_path + f"/{frame_id}.yaml"
        with open(yaml_file_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    @staticmethod
    def speed_l2_norm(speed):
        return math.sqrt(speed.x ** 2 + speed.y ** 2 + speed.z ** 2)

    def __generate_vehicle_data_of_yaml(self, vehicle):
        vehicle_color_rgb = vehicle.attributes.get("color")
        if vehicle_color_rgb is None:
            vehicle_color_rgb = "None"

        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation

        vehicle_bounding_box = vehicle.bounding_box
        vehicle_center = vehicle_bounding_box.location
        vehicle_extent = vehicle_bounding_box.extent

        vehicle_speed_l2_norm = Cav.speed_l2_norm(vehicle.get_velocity())

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

    def __generate_camera_data_of_yaml(self):
        all_camera_data = {}
        for i in range(0, 4):
            camera_name = f"camera{i}"
            camera_transform = self.__sensors[camera_name].get_transform()
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
