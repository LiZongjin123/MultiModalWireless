import os
import queue
import carla

class Rsu:
    def __init__(self, road_sign, index, sensors_config, root_dir_path):
        self.__blueprint_library = road_sign.get_world().get_blueprint_library()
        self.__world = road_sign.get_world()
        self.__sensors = {}
        self.__save_dir_path = root_dir_path + f"/rsu{index}"
        self.__sensors_config = sensors_config
        self.__sensor_queues = {}
        self.__road_sign = road_sign

    def init(self, need_attach_sensors):
        os.makedirs(self.__save_dir_path, exist_ok=True)
        if need_attach_sensors:
            self.__attach_sensors()

    def __attach_sensors(self):
        self.__attach_sensor("camera")
        self.__attach_sensor("depth_camera")
        self.__attach_sensor("lidar")
        self.__attach_sensor("radar")

    def __attach_sensor(self, sensor_name):
        sensor_config = self.__sensors_config["rsu"][sensor_name]
        sensor_location = carla.Location(x=sensor_config["transform"]["location"]["x"],
                                         y=sensor_config["transform"]["location"]["y"],
                                         z=sensor_config["transform"]["location"]["z"])
        sensor_rotation = carla.Rotation(yaw=sensor_config["transform"]["rotation"]["yaw"],
                                         pitch=sensor_config["transform"]["rotation"]["pitch"],
                                         roll=sensor_config["transform"]["rotation"]["roll"])

        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        sensor_blueprint = self.__blueprint_library.find(sensor_config["blueprint"])
        sensor_attributes = sensor_config["attributes"]
        for key, value in sensor_attributes.items():
            sensor_blueprint.set_attribute(key, str(value))

        sensor = self.__world.spawn_actor(sensor_blueprint, sensor_transform, attach_to=self.__road_sign)
        self.__sensor_queues[sensor_name] = queue.Queue()
        sensor.listen(self.__sensor_queues[sensor_name].put)
        self.__sensors[sensor_name] = sensor

    def destroy(self):
        for sensor in self.__sensors.values():
            sensor.stop()
            sensor.destroy()
        self.__road_sign.destroy()

    # def save_data(self, vehicles, cavs, rsus):
    #     self.__save_image()
    #     self.__save_lidar_to_pcd()
    #     self.__save_numeric_data_as_yaml(vehicles, cavs)


