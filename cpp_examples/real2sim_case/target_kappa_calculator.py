import pickle
import pandas
import numpy as np
import matplotlib.pyplot as plt


def compute_kappa(v123):
    v1 = v123[0]
    v2 = v123[1]
    v3 = v123[2]
    t0 = v2 - v1
    t0 /= np.linalg.norm(t0)
    t1 = v3 - v2
    t1 /= np.linalg.norm(t1)
    return 2.0 * np.cross(t0, t1) / (1.0 + t0.dot(t1))


data = pandas.read_csv("soft_limb_data.csv")

tag_ids_x = ["tagid_x_10{}".format(i) for i in range(1, 10)]
tag_ids_y = ["tagid_y_10{}".format(i) for i in range(1, 10)]

x = np.asarray(data.loc[7:, tag_ids_x])
z = -np.asarray(data.loc[7:, tag_ids_y])

x -= x[0, 0]
z -= z[0, 0]

# Time step, coordinate, (x, z)
pixel_coordinates = np.zeros((x.shape[0], x.shape[1], 2))
pixel_coordinates[:, :, 0] = x
pixel_coordinates[:, :, 1] = z

kappas = []
for i, ts in enumerate(pixel_coordinates):
    kappa1 = (compute_kappa(ts) + compute_kappa(ts[1:]) +
              compute_kappa(ts[2:])) / 3
    kappa2 = (compute_kappa(ts[4:]) + compute_kappa(ts[5:]) +
              compute_kappa(ts[6:])) / 3
    kappas.append((kappa1, kappa2))

    # plt.plot(pixel_coordinates[i, :, 0], pixel_coordinates[i, :, 1], zorder=0)
    # plt.scatter(x[i, :4], z[i, :4], zorder=1)
    # plt.scatter(x[i, 4], z[i, 4], zorder=1)
    # plt.scatter(x[i, 5:], z[i, 5:], zorder=1)
    # plt.axis('equal')
    # plt.show()
    # plt.savefig("snap{}.png".format(i))
    # plt.clf()

with open("target_kappas.pkl", "wb") as f:
    pickle.dump(kappas, f)
