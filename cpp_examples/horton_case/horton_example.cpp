#include "robot_description.h"

extern std::ofstream logging_output_file;  // defined in main.cpp

/*
 * 2D Horton Robot Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void getRobotDescription(int argc, char** argv, const std::shared_ptr<SoftRobots>& soft_robots,
                         const std::shared_ptr<ForceContainer>& forces,
                         std::shared_ptr<BaseLogger>& logger, SimParams& sim_params,
                         RenderParams& render_params) {

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
    soft_robots->addLimb(Vec3(-0.10, 0.00, 0.00), Vec3(-0.10, 0.00, 0.10), n, density, radius,
                         young_mod, poisson, mu);
    soft_robots->addLimb(Vec3(0.00, 0.00, 0.00), Vec3(0.00, 0.00, 0.10), n, density, radius,
                         young_mod, poisson, mu);
    soft_robots->addLimb(Vec3(0.10, 0.00, 0.00), Vec3(0.10, 0.00, 0.10), n, density, radius,
                         young_mod, poisson, mu);
    soft_robots->addLimb(Vec3(-0.10, 0.00, 0.10), Vec3(0.00, 0.00, 0.10), n, density, radius,
                         young_mod, poisson, mu);
    soft_robots->addLimb(Vec3(0.10, 0.00, 0.10), Vec3(0.00, 0.00, 0.10), n, density, radius,
                         young_mod, poisson, mu);

    // Create joints and connect appropriately
    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 3, 0);
    soft_robots->createJoint(1, -1);
    soft_robots->addToJoint(1, 3, -1);
    soft_robots->addToJoint(1, 4, -1);
    soft_robots->createJoint(2, -1);
    soft_robots->addToJoint(2, 4, 0);

    // Add gravity
    Vec3 gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(std::make_shared<GravityForce>(soft_robots, gravity_vec));

    // Add floor contact
    double delta = 5e-4;
    double nu = 5e-3;
    double floor_z = -0.015;
    forces->addForce(std::make_shared<FloorContactForce>(soft_robots, delta, nu, floor_z));

    // Add kappa_bar controller
    std::string phi_ctrl_file_path =
        "src/controllers/openloop_control_trajectories/horton_controller.csv";
    soft_robots->addController(
        std::make_shared<OpenLoopUniformKappaBarController>(soft_robots, phi_ctrl_file_path));
}
