import carla


class Utils:
    @staticmethod
    def transform(transform_config):
        location = carla.Location(x=transform_config["location"]["x"],
                                  y=transform_config["location"]["y"],
                                  z=transform_config["location"]["z"])
        rotation = carla.Rotation(roll=transform_config["rotation"]["roll"],
                                  pitch=transform_config["rotation"]["pitch"],
                                  yaw=transform_config["rotation"]["yaw"])
        transform = carla.Transform(location, rotation)
        return transform
