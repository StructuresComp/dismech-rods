"""
    Helix Under Gravity Example

    This is a pybind version of helix_example.cpp

"""
import numpy as np
import sys
import py_dismech
from pathlib import Path
from functools import partial

sim_manager = py_dismech.SimulationManager()

soft_robots = sim_manager.soft_robots
sim_params = sim_manager.sim_params
render_params = sim_manager.render_params
create_joint = sim_manager.soft_robots.createJoint
add_to_joint = sim_manager.soft_robots.addToJoint
add_force = sim_manager.forces.addForce

############################

sim_params.dt = 5e-3
sim_params.sim_time = 10
sim_params.dtol = 1e-3
sim_params.integrator = py_dismech.IMPLICIT_MIDPOINT

render_params.render_scale = 5.0
render_params.render_per = 5

# Read vertices describing helical shape from a file
vertices = np.loadtxt(
    Path(__file__).parents[2] /
    'cpp_examples/helix_case/helix_configuration.txt')

# Create the helix limb with custom configuration
radius = 5e-3
young_mod = 1e7
density = 1273.52
poisson = 0.5

add_limb = partial(sim_manager.soft_robots.addLimb,
                   rho=density,
                   rod_radius=radius,
                   youngs_modulus=young_mod,
                   poisson_ratio=poisson)

# Add the helical structure as a sequential series of vertices
add_limb(vertices)

# Fix the top end of the helix (locking the first node)
soft_robots.lockEdge(0, 0)

# Add gravity
gravity_force = py_dismech.GravityForce(soft_robots, np.array([0.0, 0.0,
                                                               -9.8]))
add_force(gravity_force)

# Initialize and run the simulation
sim_manager.initialize(sys.argv)
while not sim_manager.simulation_completed():
    sim_manager.step_simulation()
