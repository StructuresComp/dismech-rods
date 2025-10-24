import numpy as np
import os
import pickle
import pandas
import subprocess
from tqdm import tqdm
import matplotlib.pyplot as plt


def compute_kappa(xz):
    kappa = 0
    # Hardcoded for n = 25!!!
    coords = [[0, 6, 12], [6, 12, 18], [12, 18, 24]]

    for x, y, z in coords:
        v1 = xz[x]
        v2 = xz[y]
        v3 = xz[z]
        t0 = v2 - v1
        t0 /= np.linalg.norm(t0)
        t1 = v3 - v2
        t1 /= np.linalg.norm(t1)
        kappa += 2.0 * np.cross(t0, t1) / (1.0 + t0.dot(t1))
    kappa /= 3
    return kappa


def change_kappabar(
    kappabar1,
    kappabar2,
    controller_path="src/controllers/openloop_control_trajectories/real2sim_controller.csv"
):

    phi1 = np.rad2deg(2 * np.arctan(kappabar1 / 2))
    phi2 = np.rad2deg(2 * np.arctan(kappabar2 / 2))

    data = pandas.read_csv(controller_path)

    data.loc[1, "kappabar_phi_01"] = phi1
    data.loc[1, "kappabar_phi_11"] = phi2

    data.to_csv(controller_path, index=False)


def compute_current_kappas(logfile_path="log_files/real2sim/"):
    fns = os.listdir(logfile_path)
    fn = fns[-1]  # we want the latest one

    last_config = np.loadtxt(logfile_path + fn, delimiter=",")[-1]

    os.remove(logfile_path + fn)

    last_config = last_config[1:]  # get rid of time stamp
    all_x = last_config[::3]
    all_z = last_config[2::3]

    limb1 = np.zeros((n, 2))
    limb2 = np.zeros((n, 2))

    limb1[:, 0] = all_x[:n]
    limb1[:, 1] = all_z[:n]
    limb2[:, 0] = all_x[n:]
    limb2[:, 1] = all_z[n:]

    # plt.plot(limb1[:, 0], limb1[:, 1])
    # plt.plot(limb2[:, 0], limb2[:, 1])
    # plt.axis('equal')
    # plt.show()

    kappa1 = compute_kappa(limb1)
    kappa2 = compute_kappa(limb2)
    return np.array([kappa1, kappa2])


env = {**os.environ, "OMP_NUM_THREADS": str(1)}


def get_lambda_from_sim(a_kappa1, a_kappa2, t_kappa):
    change_kappabar(a_kappa1, a_kappa2)
    subprocess.call(["./disMech"], env=env)
    res_kappa = compute_current_kappas()
    return np.sum(np.abs(t_kappa - res_kappa))


with open("examples/real2sim_case/target_kappas.pkl", "rb") as f:
    target_kappas = pickle.load(f)

n = 25

# Starting seed
change_kappabar(0.2, 0.2)

a_kappa = np.zeros((2, ))
a_kappa[0] = -0.52

delta = 0.005  # correlates to 0.57 degrees

solved_phis = []
start_from = 0

alpha = 0.1
for i, (t_kappa1, t_kappa2) in enumerate(tqdm(target_kappas)):
    # if i < start_from: continue

    t_kappa = np.array([t_kappa1, t_kappa2])

    alpha = 0.1  # gradient descent step size

    prev_lambda = np.inf
    llambda = np.inf

    iter = 0

    # Gradient descent loop
    while True:

        a_kappa1, a_kappa2 = a_kappa

        llambda = get_lambda_from_sim(a_kappa1, a_kappa2, t_kappa)

        if llambda > prev_lambda:
            alpha *= 0.5
        # if iter > 15:
        #     alpha = 0.05
        #     iter = 0
        prev_lambda = llambda

        print("LLAMBDA: ", llambda)

        # some kind of tolerance
        if llambda < 2.5e-3:
            print("TOLERANCE REACHED!!!")
            break

        # Compute finite difference
        dllambda_dkappa = np.zeros((2, ))

        # ∂λ/∂κ_action,1
        lambda1 = get_lambda_from_sim(a_kappa1 + delta, a_kappa2, t_kappa)
        dllambda_dkappa[0] = lambda1 - llambda

        # ∂λ/∂κ_action,2
        lambda2 = get_lambda_from_sim(a_kappa1, a_kappa2 + delta, t_kappa)
        dllambda_dkappa[1] = lambda2 - llambda

        dllambda_dkappa /= delta

        # Gradient descent update for action kappa
        a_kappa = np.array([a_kappa1, a_kappa2])
        a_kappa -= alpha * dllambda_dkappa

        iter += 1

    phis = np.rad2deg(2 * np.arctan(a_kappa / 2))
    # Save individuals just in case we don't make it to the end
    np.savetxt("examples/real2sim_case/solved_phis/phi_t{}".format(i), phis)
    solved_phis.append(phis)

with open("examples/real2sim_case/solved_phis.pkl", "wb") as f:
    pickle.dump(solved_phis, f)
