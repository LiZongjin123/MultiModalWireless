import sys
root_path = ".."
sys.path.append(root_path)

from package.carla_simulation import CarlaSimulation

config_path = "../config.yaml"
carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.set_map()
carla_simulation.set_weather()
carla_simulation.set_spectator_at_spawn_center()
carla_simulation.set_asynchronous_running_mode()