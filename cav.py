import os
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
        self.image_queues = [queue.Queue() for _ in range(0, 4)]

    def init(self):
        os.makedirs(self.__save_dir_path, exist_ok=True)
        is_world_synchronous = self.__world.get_settings().synchronous_mode
        if is_world_synchronous:
            self.__attach_sensors()

    def destroy(self):
        for sensor in self.__sensors.values():
            sensor.stop()
            sensor.destroy()
        self.__vehicle.destroy()

    def __attach_camera(self, index, camera_config):
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
        camera.listen(self.image_queues[index].put)
        self.__sensors[f"camera{index}"] = camera

    def save_image(self):
        for i in range(0, 4):
            image = self.image_queues[i].get()
            image.save_to_disk(f"{self.__save_dir_path}/{image.frame}_camera{i}.png")

    def warmup(self):
        for i in range(0, 4):
            image = self.image_queues[i].get()

    def __attach_cameras(self, camera_config):
        for index in range(0, 4):
            self.__attach_camera(index, camera_config)

    def set_autopilot(self, is_enabled, *args, **kwargs):
        self.__vehicle.set_autopilot(is_enabled, *args, **kwargs)

    def __attach_sensors(self):
        self.__attach_cameras(self.__sensors_config["cav"]["camera"])

