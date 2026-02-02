from simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation("./config.yaml")
    simulation.init()
    simulation.set_running_mode(True)
    simulation.set_weather()
    simulation.generate_vehicles()
    simulation.run_in_synchronous_mode()
    simulation.destroy_resource()
    simulation.set_running_mode(False)
