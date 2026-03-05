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
carla_simulation.print_current_spectator_transform()