#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * 2D Horton Robot Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params,
                           renderParams& render_params) {

    sim_params.dt = 2.5e-4;
    sim_params.sim_time = 5;
    sim_params.enable_2d_sim = true;
    sim_params.adaptive_time_stepping = 20;

    render_params.show_mat_frames = true;
    render_params.render_scale = 3.5;

    int n = 15;
    double radius = 0.015;
    double young_mod = 5e5;
    double density = 1180;
    double poisson = 0.5;
    double mu = 0.4;

    // Create the limbs
    soft_robots->addLimb(Vector3d(-0.10, 0.00, 0.00), Vector3d(-0.10, 0.00, 0.10), n, density, radius, young_mod, poisson, mu);
    soft_robots->addLimb(Vector3d(0.00, 0.00, 0.00), Vector3d(0.00, 0.00, 0.10), n, density, radius, young_mod, poisson, mu);
    soft_robots->addLimb(Vector3d(0.10, 0.00, 0.00), Vector3d(0.10, 0.00, 0.10), n, density, radius, young_mod, poisson, mu);
    soft_robots->addLimb(Vector3d(-0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), n, density, radius, young_mod, poisson, mu);
    soft_robots->addLimb(Vector3d(0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), n, density, radius, young_mod, poisson, mu);

    // Create joints and connect appropriately
    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 3, 0);
    soft_robots->createJoint(1, -1);
    soft_robots->addToJoint(1, 3, -1);
    soft_robots->addToJoint(1, 4, -1);
    soft_robots->createJoint(2, -1);
    soft_robots->addToJoint(2, 4, 0);

    // Add gravity
    Vector3d gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vec));

    // Add floor contact
    double delta = 5e-4;
    double nu = 5e-3;
    double floor_z = -0.015;
    forces->addForce(make_shared<floorContactForce>(soft_robots, delta, nu, floor_z));

    // Add kappa_bar controller
    string phi_ctrl_file_path = "src/controllers/openloop_control_trajectories/horton_controller.csv";
    soft_robots->addController(make_shared<openLoopUniformKappaBarController>(soft_robots, phi_ctrl_file_path));
}
