import os
import carla
import yaml
from package.cav import Cav
from package.rsu import Rsu
from package.utils import Utils

class CarlaSimulation:
    def __init__(self):
        self.__traffic_manager = None
        self.__world = None
        self.__client = None
        self.__config = None
        self.__scenario_config = None
        self.__simulation_config = None
        self.__save_dir_path = None
        self.__scene_save_dir_path = None
        self.__blueprint_library = None
        self.__vehicles = []
        self.__road_signs = []
        self.__cavs = []
        self.__rsus = []
        self.__spawn_center = None

    def init(self, config_path):
        with open(config_path, "r") as f:
            self.__config = yaml.safe_load(f)
        self.__scenario_config = self.__config["scenario"]
        self.__simulation_config = self.__config["simulation"]
        self.__spawn_center = carla.Location(x=self.__scenario_config["spawn_center"]["x"],
                                             y=self.__scenario_config["spawn_center"]["y"],
                                             z=self.__scenario_config["spawn_center"]["z"])
        self.__save_dir_path = os.path.join(self.__config["output_path"], "carla_output")
        self.__scene_save_dir_path = os.path.join(self.__save_dir_path, "scene")

        client_timeout = self.__simulation_config["client_timeout"]
        self.__client = carla.Client("localhost", 2000)
        self.__client.set_timeout(client_timeout)
        self.__traffic_manager = self.__client.get_trafficmanager(8000)

        self.__world = self.__client.get_world()
        self.__blueprint_library = self.__world.get_blueprint_library()

    def creat_save_dir(self):
        os.makedirs(self.__save_dir_path, exist_ok=True)
        os.makedirs(self.__scene_save_dir_path, exist_ok=True)

    def set_map(self):
        self.__world = self.__client.load_world(self.__scenario_config["map"])
        self.__blueprint_library = self.__world.get_blueprint_library()

    def set_weather(self):
        weather_config = self.__simulation_config["weather"]
        weather_para = carla.WeatherParameters()
        for key, value in weather_config.items():
            exec(f"weather_para.{key} = {value}")
        self.__world.set_weather(weather_para)

    def generate_vehicles(self):
        spawn_points = self.__world.get_map().get_spawn_points()
        spawn_points_sorted = sorted(spawn_points, key=lambda p: p.location.distance(self.__spawn_center))

        preferred_vehicles = self.__simulation_config["preferred_vehicles"]

        for i, preferred_vehicle in enumerate(preferred_vehicles):
            vehicle_blueprint = self.__blueprint_library.find(preferred_vehicle)
            vehicle = self.__world.spawn_actor(vehicle_blueprint, spawn_points_sorted[i])
            self.__vehicles.append(vehicle)

    def generate_road_signs(self):
        rsu_transform_configs = self.__scenario_config["rsu_transform"]

        for rsu_transform_config in rsu_transform_configs:
            road_sign_blueprint = self.__blueprint_library.find("static.prop.trafficwarning")
            rsu_transform = Utils.get_transform(rsu_transform_config)
            road_sign = self.__world.spawn_actor(road_sign_blueprint, rsu_transform)
            self.__road_signs.append(road_sign)

    def config_actors(self):
        desired_cav_ranks = self.__scenario_config["desired_cav_ranks"]
        for i in desired_cav_ranks:
            vehicle = self.__vehicles.pop(i)
            cav = Cav(vehicle, len(self.__cavs), self.__config["sensors"]["cav"], self.__save_dir_path)
            cav.init()
            self.__cavs.append(cav)

        for road_sign in self.__road_signs:
            rsu = Rsu(road_sign, len(self.__rsus), self.__config["sensors"]["rsu"], self.__save_dir_path)
            rsu.init()
            self.__rsus.append(rsu)
        self.__road_signs = []

    def set_synchronous_running_mode(self):
        settings = self.__world.get_settings()
        frame_rate = self.__simulation_config["frame_rate"]
        settings.fixed_delta_seconds = 1.0 / frame_rate
        settings.synchronous_mode = True
        self.__traffic_manager.set_synchronous_mode(True)
        self.__world.apply_settings(settings)

    def set_asynchronous_running_mode(self):
        settings = self.__world.get_settings()
        settings.fixed_delta_seconds = None
        settings.synchronous_mode = False
        self.__traffic_manager.set_synchronous_mode(False)
        self.__world.apply_settings(settings)

    def destroy_resource(self):
        for actor in self.__vehicles + self.__road_signs + self.__cavs + self.__rsus:
            actor.destroy()

    def autopilot(self, is_enabled):
        if is_enabled:
            for actor in self.__vehicles + self.__cavs:
                actor.set_autopilot(True, self.__traffic_manager.get_port())
        else:
            for actor in self.__vehicles + self.__cavs:
                actor.set_autopilot(False)

    def running(self):
        settings = self.__world.get_settings()
        synchronous_mode = settings.synchronous_mode
        try:
            if synchronous_mode:
                self.__run_in_synchronous_mode()
            else:
                while True:
                    pass
        except KeyboardInterrupt:
            pass

    def set_top_down_view(self):
        spectator_height = self.__simulation_config["spectator_height"]
        spectator_location = carla.Location(x=self.__spawn_center.x,
                                            y=self.__spawn_center.y,
                                            z=spectator_height)
        spectator_rotation = carla.Rotation(pitch=-90)
        spectator_transform = carla.Transform(spectator_location, spectator_rotation)
        spectator = self.__world.get_spectator()
        spectator.set_transform(spectator_transform)

    def set_spectator_at_spawn_center(self):
        spectator_location = carla.Location(x=self.__spawn_center.x,
                                            y=self.__spawn_center.y,
                                            z=self.__spawn_center.z)
        spectator = self.__world.get_spectator()
        spectator.set_location(spectator_location)

    def place_actor_in_spectator_place(self, actor_type: str):
        spectator = self.__world.get_spectator()
        spectator_transform = spectator.get_transform()

        spawn_point = spectator_transform

        if actor_type == "rsu":
            spawn_point.rotation.yaw += 90.0
            spawn_point.location.z = 0.0
        elif actor_type == "cav":    
            spawn_point.location.z = 0.3
        spawn_point.rotation.roll = 0.0
        spawn_point.rotation.pitch = 0.0

        if actor_type == "rsu":
            actor_blueprint = self.__blueprint_library.find('static.prop.trafficwarning')
        elif actor_type == "cav":
            actor_blueprint = self.__blueprint_library.find("vehicle.tesla.model3")
        actor = self.__world.spawn_actor(actor_blueprint, spawn_point)

        if actor_type == "rsu":
            self.__road_signs.append(actor)
        elif actor_type == "cav":
            self.__vehicles.append(actor)
        print("actor transform:")
        print(f"x: {spawn_point.location.x}, y: {spawn_point.location.y}, z: {spawn_point.location.z}")
        print(f"yaw: {spawn_point.rotation.yaw}, roll: {spawn_point.rotation.roll}, pitch: {spawn_point.rotation.pitch}")

    def print_current_spectator_transform(self):
        spectator = self.__world.get_spectator()
        spectator_transform = spectator.get_transform()
        spectator_location = spectator_transform.location
        spectator_rotation = spectator_transform.rotation
        print("spectator transform:")
        print(f"x: {spectator_location.x}, y: {spectator_location.y}, z: {spectator_location.z}")
        print(f"yaw: {spectator_rotation.yaw}, roll: {spectator_rotation.roll}, pitch: {spectator_rotation.pitch}")

    def __run_in_synchronous_mode(self):
        warmup_seconds = self.__simulation_config["warmup_seconds"]
        duration_seconds = self.__simulation_config["duration_seconds"]
        queue_timeout = self.__simulation_config["queue_timeout"]
        fixed_delta_seconds = self.__world.get_settings().fixed_delta_seconds
        time = 0
        while time < warmup_seconds:
            self.__world.tick(seconds=queue_timeout)
            self.__warmup()
            time += fixed_delta_seconds
        time = 0
        while time < duration_seconds:
            frame_id = self.__world.tick(seconds=queue_timeout)
            self.__save_data()
            print(f"frame_id: {frame_id}")
            time += fixed_delta_seconds

    def __warmup(self):
        for actor in self.__cavs + self.__rsus:
            actor.warmup()

    def __save_data(self):
        for actor in self.__cavs + self.__rsus:
            actor.save_data()
        self.__scene_yaml_data_saving()

    def __scene_yaml_data_saving(self):
        frame_id = self.__world.get_snapshot().frame
        actors = Utils.generate_yaml_data_of_actors(self.__vehicles + self.__road_signs + self.__cavs + self.__rsus)
        yaml_data = {
            "frame": frame_id,
            "actors": actors
        }

        yaml_file_path = self.__scene_save_dir_path + f"/{frame_id}.yaml"
        with open(yaml_file_path, "w", encoding="utf-8") as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
