#include "robotDescription.h"
#include <math.h>


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Dynamic Cantilever Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params) {

    sim_params.dt = 1e-3;
    sim_params.sim_time = 15;
    sim_params.dtol = 1e-2;
    sim_params.render_scale = 3.0;
    sim_params.adaptive_time_stepping = 10;

    int n = 60;
    double radius = 0.005;
    double young_mod = 3e5;
    double density = 1200;
    double poisson = 0.5;

    int num_fingers = 5;
    double dist = 0.015;

    double theta = (num_fingers - 2) * M_PI / (2 * num_fingers);
    double R = dist / 2.0 / cos(theta);
    theta = 2 * M_PI / num_fingers;

    for (int i = 0; i < num_fingers; i++) {

        double t = theta * i;
        double x = R - R * cos(t);
        double y = R * sin(t);


        soft_robots->addLimb(Vector3d(x, y, 0.0), Vector3d(x, y, -0.3), n, density, radius, young_mod, poisson);
        soft_robots->lockEdge(i, 0);
    }

    // Add gravity
    Vector3d gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vec));

    // Add viscous damping
    double viscosity = 5.0;
    forces->addForce(make_shared<dampingForce>(soft_robots, viscosity));

    // Add self-contact
    double col_limit = 1e-3;
    double delta = 5e-4;
    double k_scaler = 1e5;
    double mu = 0.4;
    double nu = 1e-3;
    forces->addForce(make_shared<contactForce>(soft_robots, col_limit, delta, k_scaler, mu, nu));

    // Add custom active entanglement controller
    double start_time = 0.0;
    double end_time = 10.0;
    soft_robots->addController(make_shared<activeEntanglementController>(soft_robots, start_time, end_time));

//    string logfile_base = "log_files/active_entanglement";
//    int logging_period = 5;
//    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
