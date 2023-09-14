import numpy as np
import matplotlib.pyplot as plt


# Load logged data
fn = "log_files/helix/node_1.00e+07.csv"
data = np.loadtxt(fn, delimiter=",")

fig = plt.figure(figsize=(15, 5), frameon=True, dpi=150)
plt.plot(data[:, 0], data[:, -1])
plt.xlabel("Sim Time [s]", fontsize=18)
plt.ylabel("Tip Position [m]", fontsize=18)
plt.grid(which="both", color="k", linestyle="--")
# plt.savefig("helix.png", bbox_inches="tight", dpi=300)
plt.show()

