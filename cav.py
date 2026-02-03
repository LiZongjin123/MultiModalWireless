import os
import struct

import carla
import queue

class Cav:
    def __init__(self, vehicle, index, sensors_config, root_dir_path):
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
        for key, value in imu_attributes.item():
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

    def save_data(self):
        self.__save_image()
        self.__save_lidar_to_pcd()