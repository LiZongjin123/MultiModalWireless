import os
import time
import carla
import yaml
from cav import Cav

class Simulation:
    def __init__(self, config_path):
        self.__world = None
        self.__client = None
        with open(config_path, "r") as f:
            self.__config = yaml.safe_load(f)

        scenario_name = self.__config["scenario"]
        self.__scenario_config = self.__config["scenarios"][scenario_name]
        self.__simulation_config = self.__config["simulation"]
        self.__seed = self.__config["simulation"]["seed"]

        self.__save_dir_path = "./" + self.__config["output"]["root_dir_template"].format(scenario_name=scenario_name,
                                                                                          seed=self.__seed)
        self.__blueprint_library = None
        self.__cavs = []
        self.__spawn_center = carla.Location(x=self.__scenario_config["spawn_center"]["x"],
                                             y=self.__scenario_config["spawn_center"]["y"],
                                             z=self.__scenario_config["spawn_center"]["z"])
        self.__spectator = None

    def init(self):
        self.__client = carla.Client("localhost", 2000)
        self.__client.set_timeout(10.0)

        self.__world = self.__client.load_world(self.__scenario_config["map"])

        os.makedirs(self.__save_dir_path, exist_ok=True)

        self.__blueprint_library = self.__world.get_blueprint_library()

        self.__spectator = self.__world.get_spectator()
        spectator_transform = carla.Transform(self.__spawn_center)
        self.__spectator.set_transform(spectator_transform)

    def set_weather(self):
        weather_config = self.__simulation_config["weather"]
        weather_para = carla.WeatherParameters()
        for key, value in weather_config.items():
            exec(f"weather_para.{key} = {value}")
        self.__world.set_weather(weather_para)

    def generate_vehicles(self):
        spawn_points = self.__world.get_map().get_spawn_points()
        spawn_points_sorted = sorted(spawn_points, key=lambda p: p.location.distance(self.__spawn_center))

        num_vehicles = self.__scenario_config["actors"]["num_vehicles"]
        preferred_vehicles = self.__simulation_config["preferred_vehicles"]
        if len(preferred_vehicles) < num_vehicles:
            raise ValueError("所需车辆数量(num_vehicles)大于列表(preferred_vehicles)中车辆数量")

        for i in range(0, num_vehicles):
            vehicle_blueprint = self.__blueprint_library.find(preferred_vehicles[i])
            vehicle = self.__world.spawn_actor(vehicle_blueprint, spawn_points_sorted[i])
            if i in self.__scenario_config["actors"]["desired_cav_ranks"]:
                self.__set_cav(vehicle)

    def __set_cav(self, vehicle):
        cav = Cav(vehicle, len(self.__cavs), self.__config["sensors"], self.__save_dir_path)
        cav.init()
        self.__cavs.append(cav)

    def set_running_mode(self, is_synchronous_mode):
        settings = self.__world.get_settings()
        if is_synchronous_mode:
            frame_rate = self.__simulation_config["frame_rate"]
            settings.fixed_delta_seconds = 1.0 / frame_rate
            settings.synchronous_mode = True
        else:
            settings.synchronous_mode = False
        self.__world.apply_settings(settings)

    def __warmup(self):
        for cav in self.__cavs:
            cav.warmup()

    def __save_data(self):
        for cav in self.__cavs:
            cav.save_image()

    def run_in_synchronous_mode(self):
        warmup_seconds = self.__simulation_config["warmup_seconds"]
        duration_seconds = self.__simulation_config["duration_seconds"]
        queue_timeout = self.__simulation_config["queue_timeout"]
        start_time = time.time()
        while time.time() - start_time < warmup_seconds:
            self.__world.tick(seconds=queue_timeout)
            self.__warmup()
        start_time = time.time()
        while time.time() - start_time < duration_seconds:
            frame_id = self.__world.tick(seconds=queue_timeout)
            self.__save_data()
            print(f"frame_id: {frame_id}")
        print(f"完成数据采集，数据存储在 {self.__save_dir_path}")

    def destroy_resource(self):
        for cav in self.__cavs:
            cav.destroy()
    
