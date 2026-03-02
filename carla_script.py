from carla_simulation import CarlaSimulation

config_path = "./config.yaml"
carla_simulation = CarlaSimulation()
carla_simulation.init(config_path)
carla_simulation.set_weather()
carla_simulation.set_running_mode()
carla_simulation.generate_vehicles()
carla_simulation.generate_road_signs()
carla_simulation.config_actors()
carla_simulation.autopilot(True)

carla_simulation.running()
carla_simulation.autopilot(False)
carla_simulation.destroy_resource()
carla_simulation.set_asynchronous_running_mode()


