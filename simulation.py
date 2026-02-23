import os
import carla
import yaml
from cav import Cav
from rsu import Rsu
from utils import Utils

class Simulation:
    def __init__(self, config_path):
        self.__traffic_manager = None
        self.__world = None
        self.__client = None
        with open(config_path, "r") as f:
            self.__config = yaml.safe_load(f)

        scenario_name = self.__config["scenario"]
        self.__scenario_config = self.__config["scenarios"][scenario_name]
        self.__simulation_config = self.__config["simulation"]
        self.__seed = self.__config["simulation"]["seed"]

        # self.__save_dir_path = "./" + self.__config["output"]["root_dir_template"].format(scenario_name=scenario_name,
        #                                                                                   seed=self.__seed)
        self.__save_dir_path = self.__config["output_path"]

        self.__scene_save_dir_path = self.__save_dir_path + "/scene"

        self.__blueprint_library = None
        self.__vehicles = []
        self.__cavs = []
        self.__rsus = []
        self.__spawn_center = carla.Location(x=self.__scenario_config["spawn_center"]["x"],
                                             y=self.__scenario_config["spawn_center"]["y"],
                                             z=self.__scenario_config["spawn_center"]["z"])
        self.__spectator = None

    def init(self):
        client_timeout = self.__simulation_config["client_timeout"]
        self.__client = carla.Client("localhost", 2000)
        self.__client.set_timeout(client_timeout)

        self.__world = self.__client.load_world(self.__scenario_config["map"])

        os.makedirs(self.__save_dir_path, exist_ok=True)
        os.makedirs(self.__scene_save_dir_path, exist_ok=True)

        self.__blueprint_library = self.__world.get_blueprint_library()

        self.__set_spectator()

        self.__traffic_manager = self.__client.get_trafficmanager(8000)

    def __set_spectator(self):
        spectator_height = self.__simulation_config["spectator_height"]

        self.__spectator = self.__world.get_spectator()
        spectator_location = carla.Location(x=self.__spawn_center.x,
                                            y=self.__spawn_center.y,
                                            z=spectator_height)
        spectator_rotation = carla.Rotation(pitch=-90)
        spectator_transform = carla.Transform(spectator_location, spectator_rotation)
        self.__spectator.set_transform(spectator_transform)

    def set_weather(self):
        weather_config = self.__simulation_config["weather"]
        weather_para = carla.WeatherParameters()
        for key, value in weather_config.items():
            exec(f"weather_para.{key} = {value}")
        self.__world.set_weather(weather_para)

    def generate_vehicles(self, need_cav_attach_sensors):
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
                self.__set_cav(vehicle, need_cav_attach_sensors)
            else:
                self.__vehicles.append(vehicle)

    def generate_rsu(self, need_attach_sensors):
        num_rsu = self.__scenario_config["actors"]["num_rsu"]
        rsu_transform_configs = self.__scenario_config["rsu_transform"]

        if num_rsu != 1:
            raise ValueError("只支持一个rsu")

        if len(rsu_transform_configs) != num_rsu:
            raise ValueError("rsu的数量（num_rsu）和rsu的transform（rsu_transform）数量不一样")

        for i in range(0, num_rsu):
            road_sign_blueprint = self.__blueprint_library.find("static.prop.trafficwarning")
            rsu_transform = Utils.get_transform(rsu_transform_configs[i])
            road_sign = self.__world.spawn_actor(road_sign_blueprint, rsu_transform)
            self.__set_rsu(road_sign, need_attach_sensors)

    def __set_cav(self, actor, need_cav_attach_sensors):
        cav = Cav(actor, len(self.__cavs), self.__config["sensors"]["cav"], self.__save_dir_path)
        cav.init(need_cav_attach_sensors)
        self.__cavs.append(cav)

    def __set_rsu(self, actor, need_attach_sensors):
        rsu = Rsu(actor, len(self.__rsus), self.__config["sensors"]["rsu"], self.__save_dir_path)
        rsu.init(need_attach_sensors)
        self.__rsus.append(rsu)

    def set_running_mode(self, is_synchronous_mode):
        settings = self.__world.get_settings()
        if is_synchronous_mode:
            frame_rate = self.__simulation_config["frame_rate"]
            settings.fixed_delta_seconds = 1.0 / frame_rate
            settings.synchronous_mode = True
            self.__traffic_manager.set_synchronous_mode(True)
        else:
            settings.synchronous_mode = False
            self.__traffic_manager.set_synchronous_mode(False)
        self.__world.apply_settings(settings)

    def __warmup(self):
        for actor in self.__cavs + self.__rsus:
            actor.warmup()

    def __save_data(self):
        actors = self.__vehicles + self.__cavs + self.__rsus
        for actor in self.__cavs + self.__rsus:
            actor.save_data(actors)
        self.__scene_yaml_data_saving()

    def run_in_synchronous_mode(self):
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
        print(f"完成数据采集，数据存储在 {self.__save_dir_path}")

    def destroy_resource(self):
        for actor in self.__vehicles + self.__cavs + self.__rsus:
            actor.destroy()

    def autopilot(self, is_enabled):
        if is_enabled:
            for actor in self.__vehicles + self.__cavs:
                actor.set_autopilot(True, self.__traffic_manager.get_port())
        else:
            for actor in self.__vehicles + self.__cavs:
                actor.set_autopilot(False)

    def __scene_yaml_data_saving(self):
        frame_id = self.__world.get_snapshot().frame
        actors = Utils.generate_yaml_data_of_actors(self.__vehicles + self.__cavs + self.__rsus)
        yaml_data = {
            "frame": frame_id,
            "actors": actors
        }

        yaml_file_path = self.__scene_save_dir_path + f"/{frame_id}.yaml"
        with open(yaml_file_path, "w", encoding="utf-8") as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
