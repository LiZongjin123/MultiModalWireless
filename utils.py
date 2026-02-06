import math

import carla


class Utils:
    @staticmethod
    def get_transform(transform_config):
        location = carla.Location(x=transform_config["location"]["x"],
                                  y=transform_config["location"]["y"],
                                  z=transform_config["location"]["z"])
        rotation = carla.Rotation(roll=transform_config["rotation"]["roll"],
                                  pitch=transform_config["rotation"]["pitch"],
                                  yaw=transform_config["rotation"]["yaw"])
        transform = carla.Transform(location, rotation)
        return transform

    @staticmethod
    def get_transforms(transform_config):
        transforms = {}
        for key, value in transform_config.items():
            location = carla.Location(x=value["location"]["x"],
                                      y=value["location"]["y"],
                                      z=value["location"]["z"])
            rotation = carla.Rotation(yaw=value["rotation"]["yaw"],
                                      pitch=value["rotation"]["pitch"],
                                      roll=value["rotation"]["roll"])
            transform = carla.Transform(location, rotation)
            transforms[key] = transform
        return transforms

    @staticmethod
    def speed_l2_norm(speed):
        return math.sqrt(speed.x ** 2 + speed.y ** 2 + speed.z ** 2)

