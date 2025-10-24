from time import time
import os
import subprocess
import multiprocessing as mp


def run_fric_test(force):
    env = {**os.environ, "OMP_NUM_THREADS": str(1)}
    subprocess.call(["./disMech", str(force)], env=env)
    print("Finished simulation with force {}".format(force))


force = list([float(x) / 10.0 for x in range(0, 110, 5)])
force.extend([float(x) * -1.0 / 10.0 for x in range(1, 110, 5)])
force.sort()

start = time()
with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run_fric_test, force)
print("All sims took: {}".format(time() - start))
