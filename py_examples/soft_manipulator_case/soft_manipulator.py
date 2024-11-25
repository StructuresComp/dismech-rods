"""
    Soft manipulator
"""

import numpy as np
import sys
import py_dismech
from functools import partial

from utils import SoftManpulatorSim


soft_manipulator = SoftManpulatorSim(dt= 5e-3, sim_time = 10, dtol = 1e-3, 
                                     integrator = py_dismech.IMPLICIT_MIDPOINT, 
                                     render_scale = 1.0, render_per = 10)

radius = 5e-3
young_mod = 1e7
density = 1273.52
poisson = 0.5

# create the vertices of the manipulator (a straight rod)
vertices = np.zeros((101, 3))
vertices[:, 2] = np.linspace(0, 1, 101)

soft_manipulator.create_limb(vertices = vertices, radius = radius, young_mod = young_mod, 
                             density = density, poisson = poisson)

soft_manipulator.lock_edge(0, 0)
soft_manipulator.createCurvatureController(num_actuators = 5)

# Initialize and run the sim
soft_manipulator.initialize(sys.argv)
time = 0.0
control_frenquency = 50 # Hz
control_period = 1.0 / control_frenquency
control_time = 0.0

while not soft_manipulator.simulation_completed():
    # generate the random control signal
    if time >= control_time:
        random_curvature_rates = np.random.uniform(low = -1.0, high = 1.0, size=(5, 2))
        control_time += control_period    
        control_signal = soft_manipulator.curvature_control(random_curvature_rates)
    else:
        control_signal = soft_manipulator.curvature_control(np.zeros((5, 2)))


    soft_manipulator.step_simulation(control_signal)
    time += soft_manipulator.sim_params.dt