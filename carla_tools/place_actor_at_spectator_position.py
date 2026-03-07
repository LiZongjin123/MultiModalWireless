import sys
root_path = ".."
sys.path.append(root_path)

from package.carla_simulation import CarlaSimulation
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("actor_type", type=str)
parser.add_argument("-c", "--config", type=str)
args = parser.parse_args()

actor_type = args.actor_type
config_path = "../config.yaml"
if args.config is not None:
    config_path = args.config

carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.place_actor_in_spectator_place(actor_type)

carla_simulation.running()

carla_simulation.destroy_resource()
