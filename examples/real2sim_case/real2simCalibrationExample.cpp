#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Real2Sim Example
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

    sim_params.dt = 1e-2;
    sim_params.sim_time = 3;
    sim_params.dtol = 1e-3;
    sim_params.enable_2d_sim = true;
    sim_params.adaptive_time_stepping = 10;

    render_params.render_scale = 3.5;
    render_params.show_mat_frames = true;

    int n = 25;
    double radius = 0.01;
    double young_mod = 1.793e6;
    double density = 1240;
    double poisson = 0.5;

    // Create two appendages
    soft_robots->addLimb(Vector3d(0.0, 0.0, 0.0), Vector3d(0.075, 0.0, 0.0), n, density, radius, young_mod, poisson);
    soft_robots->addLimb(Vector3d(0.075, 0.0, 0.0), Vector3d(0.15, 0.0, 0.0), n, density, radius, young_mod, poisson);

    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 1, 0);

    // Fix one end
    soft_robots->lockEdge(0, 0);

    // Add gravity
    Vector3d gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vec));

    // Add viscous damping
    double viscosity = 7.0;
    forces->addForce(make_shared<dampingForce>(soft_robots, viscosity));

    // Add kappa_bar controller
    string phi_ctrl_file_path = "src/controllers/openloop_control_trajectories/real2sim_controller.csv";
    soft_robots->addController(make_shared<openLoopUniformKappaBarController>(soft_robots, phi_ctrl_file_path));

    // Set logger to record nodes
    string logfile_base = "log_files/real2sim";
    int logging_period = 50;
    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
