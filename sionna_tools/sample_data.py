import sys
root_path = ".."
sys.path.append(root_path)

from package.sionna_simulation import SionnaSimulation

config_path = "../config.yaml"
sionna_simulation = SionnaSimulation()
sionna_simulation.init(config_path)
sionna_simulation.set_gpu()
sionna_simulation.creat_save_dir()
sionna_simulation.generate_channel_data()
