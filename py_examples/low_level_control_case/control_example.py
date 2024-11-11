"""
    Change the boundary conditions of a straight rod

"""
import numpy as np
import sys
import py_dismech
from pathlib import Path
from functools import partial

sim_manager = py_dismech.SimulationManager()

soft_robots = sim_manager.soft_robots # soft robot class
sim_params = sim_manager.sim_params # simulation parameters
render_params = sim_manager.render_params # render paramters
create_joint = sim_manager.soft_robots.createJoint # function
add_to_joint = sim_manager.soft_robots.addToJoint # function
add_force = sim_manager.forces.addForce # external force

############################

sim_params.dt = 5e-3
sim_params.sim_time = 10
sim_params.dtol = 1e-3
sim_params.integrator = py_dismech.IMPLICIT_MIDPOINT

render_params.render_scale = 1.0
render_params.render_per = 10

# Read vertices describing helical shape from a file
vertices = np.zeros((101, 3))
vertices[:, 2] = np.linspace(0, 1, 101)

# Create the helix limb with custom configuration
radius = 5e-3
young_mod = 1e7
density = 1273.52
poisson = 0.5

add_limb = partial(sim_manager.soft_robots.addLimb, rho=density, rod_radius=radius,
                   youngs_modulus=young_mod, poisson_ratio=poisson)

# Add the helical structure as a sequential series of vertices
add_limb(vertices)

# Fix the top end of the helix (locking the first node)
soft_robots.lockEdge(0, 0)

# Add gravity
gravity_force = py_dismech.GravityForce(soft_robots, np.array([0.0, 0.0, -9.8]))
add_force(gravity_force)

# Initialize and run the simulation
sim_manager.initialize(sys.argv)
time = 0.0
while not sim_manager.simulation_completed():
    input_dict = {"position" : np.array([[0, 0, time * 0.01, 0.0, 0.0], [0, 1, time * 0.01, 0.0, 0.0]])}
    sim_manager.step_simulation(input_dict)
    time += sim_params.dt
