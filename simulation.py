import os
import carla
import yaml
from cav import Cav

class Simulation:
    # blueprint_library = None
    # SAVE_DIR_PATH = None

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
