import os
import queue
import struct
from abc import ABC, abstractmethod
import yaml
from utils import Utils

class Mountable(ABC):
    def __init__(self, index, sensors_config, actor, save_dir_path, name):
        self._index = index
        self._name = name
        self._sensors_config = sensors_config
        self._world = actor.get_world()
        self._blueprint_library = self._world.get_blueprint_library()
        self._sensor_queues = {}
        self._sensors = {}
        self._save_dir_path = save_dir_path
        self._actor = actor

    def init(self, need_attach_sensors):
        os.makedirs(self._save_dir_path, exist_ok=True)
        if need_attach_sensors:
            self._attach_sensors()

    @abstractmethod
    def _attach_sensors(self):
        pass

    def _attach_sensor(self, sensor_name):
        sensor_config = self._sensors_config[sensor_name]
        sensor_transform_config = sensor_config["transform"]
        sensor_transforms = {}
        if "location" in sensor_transform_config:
            sensor_transforms[sensor_name] = Utils.get_transform(sensor_transform_config)
        else:
            sensor_transforms = Utils.get_transforms(sensor_transform_config)
        for sensor_name_, sensor_transform in sensor_transforms.items():
            sensor_blueprint = self._blueprint_library.find(sensor_config["blueprint"])

            sensor_attributes = sensor_config["attributes"]
            for key, value in sensor_attributes.items():
                sensor_blueprint.set_attribute(key, str(value))

            sensor = self._world.spawn_actor(sensor_blueprint, sensor_transform, attach_to=self._actor)
            self._sensor_queues[sensor_name_] = queue.Queue()
            sensor.listen(self._sensor_queues[sensor_name_].put)
            self._sensors[sensor_name_] = sensor

    def destroy(self):
        for sensor in self._sensors.values():
            sensor.stop()
            sensor.destroy()
        self._actor.destroy()

    def warmup(self):
        for sensor_queue in self._sensor_queues.values():
            data = sensor_queue.get()

    @abstractmethod
    def save_data(self, actors):
        pass

    def _lidar_saving(self):
        lidar_measurement = self._sensor_queues["lidar"].get()
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
        pcd_file_path = self._save_dir_path + f"/{frame_id}.pcd"

        with open(pcd_file_path, 'w', encoding='utf-8') as f:
            f.write(pcd_content)

    def _yaml_data_saving(self, actors):
        yaml_data_of_sensors = self._generate_yaml_data_of_sensors()
        yaml_data_of_actors = Utils.generate_yaml_data_of_actors(actors, self._actor)

        frame_id = self._world.get_snapshot().frame

        data = {
            "actor": self._name,
            "frame": frame_id,
            "sensors": yaml_data_of_sensors,
            "actors": yaml_data_of_actors
        }

        yaml_file_path = self._save_dir_path + f"/{frame_id}.yaml"
        with open(yaml_file_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    @abstractmethod
    def _generate_yaml_data_of_sensors(self):
        pass

    def _generate_yaml_data_of_all_sensor_poses(self):
        yaml_data_of_all_sensor_poses = {}
        for sensor_name, sensor in self._sensors.items():
            sensor_transform = sensor.get_transform()
            yaml_data_of_sensor_pose = Utils.generate_yaml_data_of_actor_pose(sensor_transform)
            yaml_data_of_all_sensor_poses[sensor_name] = yaml_data_of_sensor_pose
        return yaml_data_of_all_sensor_poses

    def _generate_yaml_data_of_self_actor_pose(self):
        self_actor_transform = self._actor.get_transform()
        yaml_data_of_self_actor_pose = Utils.generate_yaml_data_of_actor_pose(self_actor_transform)
        return yaml_data_of_self_actor_pose
