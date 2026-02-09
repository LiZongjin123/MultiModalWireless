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

    @staticmethod
    def generate_yaml_data_of_actor_pose(transform):
        location = transform.location
        rotation = transform.rotation

        yaml_data = {
            "location": {
                "x": location.x,
                "y": location.y,
                "z": location.z
            },
            "rotation": {
                "pitch": rotation.pitch,
                "roll": rotation.roll,
                "yaw": rotation.yaw
            }
        }
        return yaml_data

    @staticmethod
    def _generate_yaml_data_of_actor(actor):
        actor_color = actor.attributes.get("color")
        if actor_color is None:
            actor_color = "None"

        actor_transform = actor.get_transform()
        actor_pose = Utils.generate_yaml_data_of_actor_pose(actor_transform)

        actor_bounding_box = actor.bounding_box
        actor_center = actor_bounding_box.location
        actor_extent = actor_bounding_box.extent

        actor_speed_l2_norm = Utils.speed_l2_norm(actor.get_velocity())

        yaml_data_of_actor = {
            "bp_id": actor.type_id,
            "color": actor_color,
            "speed": actor_speed_l2_norm,
            "location": actor_pose["location"],
            "rotation": actor_pose["rotation"],
            "center": {
                "x": actor_center.x,
                "y": actor_center.y,
                "z": actor_center.z
            },
            "extent": {
                "x": actor_extent.x,
                "y": actor_extent.y,
                "z": actor_extent.z
            }
        }
        return yaml_data_of_actor

    @staticmethod
    def generate_yaml_data_of_actors(actors, excluded_actor=None):
        yaml_data_of_actors = {}
        for actor in actors:
            if not hasattr(actor, "id"):
                actor = actor._actor
            if (not excluded_actor is None) and (actor.id == excluded_actor.id):
                continue
            yaml_data_of_actor = Utils._generate_yaml_data_of_actor(actor)
            yaml_data_of_actors[actor.id] = yaml_data_of_actor
        return yaml_data_of_actors
