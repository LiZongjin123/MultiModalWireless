import sys
root_path = ".."
sys.path.append(root_path)

from package.sionna_simulation import SionnaSimulation

config_path = "../config.yaml"
frame_id = 60561

def print_npz_data_shape(npz_files):
    cir_a = npz_files["cir_a"]
    cir_tau = npz_files["cir_tau"]
    cfr = npz_files["cfr"]
    doppler = npz_files["doppler"]
    theta_t = npz_files["theta_t"]
    phi_t = npz_files["phi_t"]
    theta_r = npz_files["theta_r"]
    phi_r = npz_files["phi_r"]

    print("cir_a shape:", cir_a.shape)
    print("cir_tau shape:", cir_tau.shape)
    print("cfr shape", cfr.shape)
    print("doppler shape", doppler.shape)
    print("theta_t shape", theta_t.shape)
    print("phi_t shape", phi_t.shape)
    print("theta_r shape", theta_r.shape)
    print("phi_r shape", phi_r.shape)

sionna_simulation = SionnaSimulation()
sionna_simulation.init(config_path)
npz_file = sionna_simulation.get_npz_file(frame_id)
print_npz_data_shape(npz_file)
