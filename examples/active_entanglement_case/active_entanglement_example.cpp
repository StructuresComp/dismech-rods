#include "robot_description.h"
#include <math.h>

extern ofstream logging_output_file;  // defined in main.cpp

/*
 * Dynamic Cantilever Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void getRobotDescription(int argc, char** argv, const shared_ptr<SoftRobots>& soft_robots,
                         const shared_ptr<ForceContainer>& forces, shared_ptr<BaseLogger>& logger,
                         SimParams& sim_params, RenderParams& render_params) {

    sim_params.dt = 2.5e-3;
    sim_params.sim_time = 15;
    sim_params.dtol = 1e-2;
    sim_params.adaptive_time_stepping = 10;

    render_params.render_scale = 3.0;

    int n = 60;
    double radius = 0.005;
    double young_mod = 3e5;
    double density = 1200;
    double poisson = 0.5;
    double mu = 0.5;

    int num_fingers = 5;
    double dist = 0.015;

    double theta = (num_fingers - 2) * M_PI / (2 * num_fingers);
    double R = dist / 2.0 / cos(theta);
    theta = 2 * M_PI / num_fingers;

    for (int i = 0; i < num_fingers; i++) {

        double t = theta * i;
        double x = R - R * cos(t);
        double y = R * sin(t);

        soft_robots->addLimb(Vector3d(x, y, 0.0), Vector3d(x, y, -0.3), n, density, radius,
                             young_mod, poisson, mu);
        soft_robots->lockEdge(i, 0);
    }

    // Add gravity
    Vector3d gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(make_shared<GravityForce>(soft_robots, gravity_vec));

    // Add viscous damping
    double viscosity = 5.0;
    forces->addForce(make_shared<DampingForce>(soft_robots, viscosity));

    // Add self-contact
    double col_limit = 1e-3;
    double delta = 5e-4;
    double k_scaler = 1e5;
    double nu = 1e-3;
    bool friction = false;  // for friction, reduce time step to 1 ms
    bool self_contact = true;
    forces->addForce(make_shared<ContactForce>(soft_robots, col_limit, delta, k_scaler, friction,
                                               nu, self_contact));

    // Add custom active entanglement controller
    double start_time = 0.0;
    double end_time = 10.0;
    soft_robots->addController(
        make_shared<ActiveEntanglementController>(soft_robots, start_time, end_time));

    string logfile_base = "log_files/active_entanglement";
    int logging_period = 5;
    logger = make_shared<PositionLogger>(logfile_base, logging_output_file, logging_period);
}
