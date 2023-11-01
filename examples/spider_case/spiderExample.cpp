#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Spider on Incline Example
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
    sim_params.sim_time = 2;
    sim_params.ftol = 1e-3;
    sim_params.render_scale = 3.0;
    sim_params.show_mat_frames = true;
    sim_params.adaptive_time_stepping = 7;

    int n = 25;
    double radius = 5e-3;
    double young_mod = 3e6;
    double density = 1180;
    double poisson = 0.5;

    // Create the limbs
    soft_robots->addLimb(Vector3d(0, 0, 0.20), Vector3d(0, 0.00, 0.10), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0.10, 0, 0.10), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0, 0.10, 0.10), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0, -0.10, 0.10), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(-0.10, 0, 0.10), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0.10, 0, 0.10), Vector3d(0.10, 0, 0.00), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0.0, 0.10, 0.10), Vector3d(0.0, 0.10, 0.00), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0.0, -0.10, 0.10), Vector3d(0.0, -0.10, 0.00), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(-0.10, 0, 0.10), Vector3d(-0.10, 0, 0.00), n, density, radius, young_mod, poisson);

    // Create joints and connect appropriately
    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 1, 0);
    soft_robots->addToJoint(0, 2, 0);
    soft_robots->addToJoint(0, 3, 0);
    soft_robots->addToJoint(0, 4, 0);
    soft_robots->createJoint(1, -1);
    soft_robots->addToJoint(1, 5, 0);
    soft_robots->createJoint(2, -1);
    soft_robots->addToJoint(2, 6, 0);
    soft_robots->createJoint(3, -1);
    soft_robots->addToJoint(3, 7, 0);
    soft_robots->createJoint(4, -1);
    soft_robots->addToJoint(4, 8, 0);

    // This has to be called after all joints are declared
    soft_robots->setup();

    // Add gravity with a slight x-axis perturbation
    Vector3d gravity_vec(1.0, 0.0, -9.8);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vec));

    // Add floor contact
    double delta = 5e-4;
    double nu = 5e-3;
    double mu = 0.4;
    double floor_z = -0.10;
    forces->addForce(make_shared<floorContactForce>(soft_robots, delta, nu, mu, floor_z));

    string logfile_base = "log_files/spider";
    int logging_period = 20;
    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
