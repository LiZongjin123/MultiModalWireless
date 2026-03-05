import sys
root_path = ".."
sys.path.append(root_path)

from package.sionna_simulation import SionnaSimulation
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--config", type=str)
args = parser.parse_args()

config_path = "../config.yaml"
if args.config is not None:
    config_path = args.config

sionna_simulation = SionnaSimulation()
sionna_simulation.init(config_path)
sionna_simulation.set_gpu()
sionna_simulation.creat_save_dir()
sionna_simulation.generate_channel_data()
