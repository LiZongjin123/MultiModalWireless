import sys
root_path = ".."
sys.path.append(root_path)

from package.carla_simulation import CarlaSimulation
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--config", type=str)
args = parser.parse_args()

config_path = "../config.yaml"
if args.config is not None:
    config_path = args.config

carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.creat_save_dir()
carla_simulation.set_map()
carla_simulation.set_weather()
carla_simulation.set_top_down_view()
carla_simulation.set_synchronous_running_mode()
carla_simulation.generate_vehicles()
carla_simulation.generate_road_signs()
carla_simulation.config_actors()
carla_simulation.autopilot(True)

carla_simulation.running()

carla_simulation.autopilot(False)
carla_simulation.destroy_resource()
carla_simulation.set_asynchronous_running_mode()


