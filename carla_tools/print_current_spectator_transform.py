import sys
root_path = ".."
sys.path.append(root_path)

from package.carla_simulation import CarlaSimulation

config_path = "../config.yaml"
carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.print_current_spectator_transform()