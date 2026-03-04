import sys
root_path = ".."
sys.path.append(root_path)

from package.carla_simulation import CarlaSimulation

config_path = "../config.yaml"
carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.place_rsu_in_spectator_place()

carla_simulation.running()

carla_simulation.destroy_resource()
