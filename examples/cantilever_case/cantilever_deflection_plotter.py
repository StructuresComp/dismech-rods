import numpy as np
import matplotlib.pyplot as plt


# Load logged data
fn = "log_files/cantilever/node_1.00e+05.csv"
# fn = "log_files/cantilever/node_1.00e+07.csv"
data = np.loadtxt(fn, delimiter=",")

gt = np.loadtxt("examples/cantilever_case/euler_bernoulli_data_E=1e5_T=100s.txt")
# gt = np.loadtxt("examples/cantilever_case/euler_bernoulli_data_E=1e7_T=20s.txt")

fig = plt.figure(figsize=(15, 5), frameon=True, dpi=150)
ax = fig.add_subplot(111)
ax.plot(data[:, 0], data[:, -1], lw=2.0, label="DisMech Sim")
ax.plot(gt[:, 0], gt[:, -1], lw=2.0, label="Euler Bernoulli Beam Theory")
ax.set_xlabel("Sim Time [s]", fontsize=18)
ax.set_ylabel("Tip Position [m]", fontsize=18)
ax.grid(which="both", color="k", linestyle="--")
ax.legend()
# plt.savefig("cantilever.png", bbox_inches="tight", pad_inches=0, dpi=300)
plt.show()

