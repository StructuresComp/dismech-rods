import numpy as np
from matplotlib import pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d
import sys

## load the vertices from text file
vertices = np.loadtxt(Path(__file__).resolve().parents[0] / 'clip_configuration.txt')
# calculate the culumative sum of the vertices
distances = np.sqrt(np.sum(np.diff(vertices, axis=0)**2, axis=1))
cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

# Interpolate to get equal distances
num_points = 100  # Number of points for interpolation
equal_distances = np.linspace(0, cumulative_distances[-1], num_points)

# Interpolate to get equal distances
num_points = 100  # Number of points for interpolation
equal_distances = np.linspace(0, cumulative_distances[-1], num_points)

interp_func_x = interp1d(cumulative_distances, vertices[:, 0], kind='linear')
interp_func_y = interp1d(cumulative_distances, vertices[:, 1], kind='linear')

interpolated_vertices = np.vstack((interp_func_x(equal_distances), interp_func_y(equal_distances))).T

vertices = np.hstack((interpolated_vertices[:, 0].reshape(-1, 1),
                      np.zeros_like(interpolated_vertices[:, 0]).reshape(-1, 1),
                      interpolated_vertices[:, 1].reshape(-1, 1)))

dis = np.diff(vertices, axis=0)
dis = np.sqrt(np.sum(dis**2, axis=1))
vertices = vertices/100.0


plt.plot(vertices[:, 0], vertices[:, 2])
plt.axis('equal')
plt.show()

## define the simulator
import py_dismech
from functools import partial

sim_manager = py_dismech.SimulationManager()

soft_robot = sim_manager.soft_robots
sim_params = sim_manager.sim_params
render_params = sim_manager.render_params
create_joint = sim_manager.soft_robots.createJoint
add_to_joint = sim_manager.soft_robots.addToJoint
add_force = sim_manager.forces.addForce


##################################
sim_params.dt = 1e-3
sim_params.sim_time = 100
sim_params.dtol = 1e-4
sim_params.integrator = py_dismech.IMPLICIT_MIDPOINT
sim_params.line_search = py_dismech.WOLFE

render_params.render_scale = 5.0
render_params.render_per = 5

# Read vertices describing helical shape from a file
# vertices = np.zeros((101, 3))
# vertices[:, 2] = np.linspace(0, 1, 101)

# create the clip body as a limb
radius = 1.6e-3
young_mod = 1e7
density = 1000.0
poisson = 0.33

add_limb = partial(sim_manager.soft_robots.addLimb, rho=density, rod_radius=radius,
                   youngs_modulus=young_mod, poisson_ratio=poisson)
add_limb(vertices)


soft_robot.lockNode(0, 1)
soft_robot.lockTwist(0, 0)
soft_robot.lockNode(0, 98)
soft_robot.lockTwist(0, 98)

# get the middle point
middle_point1 = (vertices[1, :] + vertices[-2, :])/2.0
middle_point2 = (vertices[1, :] + vertices[-2, :])/2.0
middle_point1[1] = 0.001
middle_point2[1] = -0.001


v1 = - vertices[1, :] + middle_point1
v1 = v1/np.linalg.norm(v1)
v2 = - vertices[-2, :] + middle_point2
v2 = v2/np.linalg.norm(v2)

v1 *= 0.01
v2 *= 0.01

# Initialize and run the simulation
sim_manager.initialize(sys.argv)

while not sim_manager.simulation_completed():
    # compute the boundary changes
    sim_manager.step_simulation()

    monitor_v1 = soft_robot.limbs[0].getVertexPos(1)
    monitor_v2 = soft_robot.limbs[0].getVertexPos(98)

    montior_vm = soft_robot.limbs[0].getVertexPos(50)
    print(montior_vm)

    if np.linalg.norm(monitor_v1 - middle_point1) < 1e-4 and np.linalg.norm(monitor_v2 - middle_point2) < 1e-4:
        # inputs = np.array([[0, 0, middle_point1[0], middle_point1[1], middle_point1[2]],
        #                    [0, 99, middle_point2[0], middle_point2[1], middle_point2[2]]]).reshape(-1, 5)
        # input_dict = {"position" : inputs}
        sim_manager.step_simulation()
    else:
        dx1 = v1 * sim_params.dt
        dx2 = v2 * sim_params.dt

        inputs = np.array([[0, 1, dx1[0], dx1[1], dx1[2]],
                           [0, 98, dx2[0], dx2[1], dx2[2]]]).reshape(-1, 5)
        input_dict = {"dposition" : inputs}
        sim_manager.step_simulation(input_dict)

    # print(soft_robot.limbs[0].getVertexPos(0))
    # print(soft_robot.limbs[0].getVertexPos(0))
