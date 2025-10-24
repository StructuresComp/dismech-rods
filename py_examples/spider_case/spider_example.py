"""
    Spider on Incline Example

    This is a pybind version of spider_example.cpp
"""

import numpy as np
import sys
import py_dismech
from functools import partial

sim_manager = py_dismech.SimulationManager()

soft_robots = sim_manager.soft_robots
sim_params = sim_manager.sim_params
render_params = sim_manager.render_params
create_joint = sim_manager.soft_robots.createJoint
add_to_joint = sim_manager.soft_robots.addToJoint
add_force = sim_manager.forces.addForce

SIM_FAST = True

############################

sim_params.sim_time = 2
sim_params.ftol = 1e-3

render_params.render_scale = 5.0
render_params.show_mat_frames = True
render_params.renderer = py_dismech.OPENGL

if SIM_FAST:
    sim_params.dt = 2.5e-3
    sim_params.max_iter.num_iters = 15
    sim_params.max_iter.terminate_at_max = False
    delta = 5e-3
    nu = 1e-2
else:
    sim_params.dt = 1e-3
    sim_params.adaptive_time_stepping = 7
    delta = 5e-4
    nu = 5e-3

n = 25
radius = 5e-3
young_mod = 3e6
density = 1180
poisson = 0.5
mu = 0.4
add_limb = partial(sim_manager.soft_robots.addLimb,
                   num_nodes=n,
                   rho=density,
                   rod_radius=radius,
                   youngs_modulus=young_mod,
                   poisson_ratio=poisson,
                   mu=mu)

add_limb(np.array([0.00, 0.00, 0.20]), np.array([0.00, 0.00, 0.10]))
add_limb(np.array([0.00, 0.00, 0.10]), np.array([0.10, 0.00, 0.10]))
add_limb(np.array([0.00, 0.00, 0.10]), np.array([0.00, 0.10, 0.10]))
add_limb(np.array([0.00, 0.00, 0.10]), np.array([0.00, -0.10, 0.10]))
add_limb(np.array([0.00, 0.00, 0.10]), np.array([-0.10, 0.00, 0.10]))
add_limb(np.array([0.10, 0.00, 0.10]), np.array([0.10, 0.00, 0.00]))
add_limb(np.array([0.00, 0.10, 0.10]), np.array([0.00, 0.10, 0.00]))
add_limb(np.array([0.00, -0.10, 0.10]), np.array([0.00, -0.10, 0.00]))
add_limb(np.array([-0.10, 0.00, 0.10]), np.array([-0.10, 0.00, 0.00]))

create_joint(0, -1)
add_to_joint(0, 1, 0)
add_to_joint(0, 2, 0)
add_to_joint(0, 3, 0)
add_to_joint(0, 4, 0)
create_joint(1, -1)
add_to_joint(1, 5, 0)
create_joint(2, -1)
add_to_joint(2, 6, 0)
create_joint(3, -1)
add_to_joint(3, 7, 0)
create_joint(4, -1)
add_to_joint(4, 8, 0)

# Add gravity with a slight x-axis perturbation
gravity_force = py_dismech.GravityForce(soft_robots, np.array([1.0, 0.0,
                                                               -9.8]))
add_force(gravity_force)

# Add floor contact
floor_z = -0.10
floor_contact_force = py_dismech.FloorContactForce(soft_robots, delta, nu,
                                                   floor_z)
add_force(floor_contact_force)

# Initialize and run the simulation
sim_manager.initialize(sys.argv)
while not sim_manager.simulation_completed():
    sim_manager.step_simulation()
