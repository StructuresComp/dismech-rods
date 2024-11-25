import py_dismech
from functools import partial
import numpy as np


class SoftManpulatorSim(py_dismech.SimulationManager):
    def __init__(self, dt = 5e-3, sim_time = 10, dtol = 1e-3, 
                 integrator = py_dismech.IMPLICIT_MIDPOINT, 
                 render_scale = 1.0, render_per = 10):
        super().__init__()
        self.sim_params.dt  = dt
        self.sim_params.dt = dt
        self.sim_params.sim_time = sim_time
        self.sim_params.dtol = dtol
        self.sim_params.integrator = integrator
        self.render_params.render_scale = render_scale
        self.render_params.render_per = render_per

    def create_limb(self, vertices, radius, young_mod, density, poisson):
        add_limb = partial(self.soft_robots.addLimb, rho=density, rod_radius=radius,
                           youngs_modulus=young_mod, poisson_ratio=poisson)
        add_limb(vertices)

    def createCurvatureController(self, num_actuators = 5):
        # actuator is defined by the starting nodes to the ending nodes
        nv = self.soft_robots.limbs[0].getNv()
        self.actuators = []
        gap = nv // num_actuators
        for i in range(num_actuators):
            start_node = i * gap
            end_node = (i + 1) * gap
            self.actuators.append([start_node, end_node])

        self.actuators[0][0] = 1
        if self.actuators[-1][1] >= nv:
            self.actuators[-1][1] = nv - 2


    def curvature_control(self, curvature_rates):
        control_signal = []
        for i, actuator in enumerate(self.actuators):
            for k in range(actuator[0], actuator[1]):
                control_signal.append([0, k, *(self.sim_params.dt * curvature_rates[i, :]).tolist()])
        control_signal = np.array(control_signal).reshape(-1, 4)
        control_signal = {"curvature" : control_signal}

        return control_signal

    def lock_edge(self, edge, node):
        self.soft_robots.lockEdge(edge, node)

    def add_gravity(self, gravity):
        gravity_force = py_dismech.GravityForce(self.soft_robots, gravity)
        self.add_force(gravity_force)

    # def step_simulation(self, input_dict):
    #     self.step_simulation(input_dict)