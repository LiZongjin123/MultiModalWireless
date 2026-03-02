from sionna_simulation import SionnaSimulation

config_path = "./config.yaml"
sionna_script = SionnaSimulation()
sionna_script.init(config_path)
sionna_script.generate_channel_data()
