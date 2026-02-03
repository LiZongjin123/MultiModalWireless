from simulation import Simulation

if __name__ == "__main__":
    is_synchronous_mode = False
    simulation = Simulation("./config.yaml")
    simulation.init()
    simulation.set_weather()
    simulation.generate_vehicles()
    if is_synchronous_mode:
        simulation.set_running_mode(True)
        simulation.autopilot(True)
        simulation.run_in_synchronous_mode()
        simulation.autopilot(False)
        simulation.destroy_resource()
        simulation.set_running_mode(False)
    else:
        simulation.set_running_mode(False)
        simulation.autopilot(True)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            simulation.autopilot(False)
            simulation.destroy_resource()

