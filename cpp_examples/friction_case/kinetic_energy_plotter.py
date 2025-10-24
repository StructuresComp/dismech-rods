import os
import numpy as np
import matplotlib.pyplot as plt

# Plot theoretical results
fig = plt.figure(figsize=(12, 6))
u = np.linspace(-10.6, 10.5, 100)
af = 1**2 / 2 * (np.abs(u) - 0.4 * 9.8)**2
mask = (np.abs(u) - 0.4 * 9.8) < 0
af[mask] = 0
plt.plot(u, af, linewidth=2, label="theoretical", zorder=0)

# Construct Mass Array
ne = 25
# rod length * pi * rod radius^2 * rho
mass = 1 * np.pi * 0.025**2 * 509.2985
dm = mass / ne
mass_array = np.zeros((ne + 1, 1))
mass_array[1:-1] = dm
mass_array[[0, -1]] = 0.5 * dm

# Compute kinetic energies from simulation
pull_force_kes = {}
data_directory = "log_files/friction_case"
for f in os.listdir(data_directory):
    # Grab the external force value from file name
    force = (f.split(".csv")[0]).split("velocities_")[1]

    # Grab the x velocities for the last time step
    u = np.loadtxt(data_directory + "/" + f,
                   delimiter=",")[-1, 1:][0::3].reshape((-1, 1))

    ke = 0.5 * mass_array * u**2

    pull_force_kes[float(force)] = ke.sum()

# Plot the results
plt.scatter(pull_force_kes.keys(),
            pull_force_kes.values(),
            c="#ff7f0e",
            s=100,
            marker="*",
            label="dismech",
            zorder=1)
plt.xlabel("Pull Force [N]", fontsize=18)
plt.ylabel("Kinetic Energy [J]", fontsize=18)
plt.legend(fontsize=18)
plt.show()
# plt.savefig("friction_example.png", bbox_inches="tight", pad_inches=0, dpi=300)
