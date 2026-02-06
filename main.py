from simulation import Simulation

if __name__ == "__main__":
    is_synchronous_mode = True
    simulation = Simulation("./config.yaml")
    simulation.init()
    simulation.set_weather()
    simulation.set_running_mode(is_synchronous_mode)
    simulation.generate_vehicles(is_synchronous_mode)
    simulation.generate_rsu(is_synchronous_mode)
    simulation.autopilot(True)
    try:
        if is_synchronous_mode:
            simulation.run_in_synchronous_mode()
        else:
            while True:
                pass
    except KeyboardInterrupt:
        pass

    simulation.autopilot(False)
    simulation.destroy_resource()
    simulation.set_running_mode(False)


