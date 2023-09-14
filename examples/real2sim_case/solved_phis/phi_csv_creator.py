import os
import pandas
import numpy as np


files = os.listdir()
files.remove("phi_csv_creator.py")
files.remove("solved_phis.csv")
files.sort(key=lambda x : int(x.split("phi_t")[1]))

phis = []
for f in files:
    phis.append(np.loadtxt(f))

phis = np.asarray(phis)
data = np.zeros((phis.shape[0], 5))
data[:, 2] = phis[:, 0]
data[:, 4] = phis[:, 1]
data[:, 0] = 0
data[1:, 0] = np.linspace(1.5, 11.5, phis.shape[0]-1)

data = pandas.DataFrame(data, columns=["Simulation time (sec)",
                                       "kappa_bar_phi_00",
                                       "kappa_bar_phi_01",
                                       "kappa_bar_phi_10",
                                       "kappa_bar_phi_11"])

data.to_csv("solved_phis.csv", index=False, lineterminator=",\n")
