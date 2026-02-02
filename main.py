from simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation("./config.yaml")
    simulation.init()
    simulation.set_weather()
    simulation.generate_vehicles()
