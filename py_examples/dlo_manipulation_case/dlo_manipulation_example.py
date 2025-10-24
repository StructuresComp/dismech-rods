"""
    Deformable object manipulation example.
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

############################

sim_params.sim_time = 60
sim_params.ftol = 1e-3
sim_params.dt = 2.5e-3

render_params.render_scale = 1.0
render_params.show_mat_frames = True
render_params.renderer = py_dismech.OPENGL

n = 100
radius = 1.6e-3
young_mod = 1.8e5
density = 1180
poisson = 0.5
add_limb = partial(sim_manager.soft_robots.addLimb,
                   num_nodes=n,
                   rho=density,
                   rod_radius=radius,
                   youngs_modulus=young_mod,
                   poisson_ratio=poisson)

# Create a single rod.
add_limb(np.array([0.00, 0.00, 0.20]), np.array([1.00, 0.00, 0.20]))

gravity_force = py_dismech.GravityForce(soft_robots, np.array([0.0, 0.0,
                                                               -9.8]))
add_force(gravity_force)

# Hold the two ends of the rod.
soft_robots.lockEdge(0, 0)
soft_robots.lockEdge(0, 98)

# Initialize and run the simulation
curr_time = 0.0
pos_iter = 0
twist_iter = 0
sim_manager.initialize(sys.argv)
while not sim_manager.simulation_completed():
    # First, twist the rod on one side with constant acceleration.
    if 5.0 < curr_time < 10.0:
        twist_iter += 1
        theta_change = {"delta_theta": np.array([0, 0, twist_iter * 1e-5])}
        sim_manager.step_simulation(theta_change)
    # Then, move one rod end closer to the other with constant acceleration.
    elif 10.0 < curr_time < 15.0:
        pos_iter += 1
        pos_change = {
            "delta_position":
            np.array([[0, 0, pos_iter * 2.5e-7, 0.0, 0.0],
                      [0, 1, pos_iter * 2.5e-7, 0.0, 0.0]])
        }
        sim_manager.step_simulation(pos_change)
    else:
        sim_manager.step_simulation()
    curr_time += sim_params.dt
