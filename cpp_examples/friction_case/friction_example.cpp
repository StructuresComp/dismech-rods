#include "robot_description.h"

extern std::ofstream logging_output_file;  // defined in main.cpp

/*
 * Axial Friction Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void getRobotDescription(int argc, char** argv, const std::shared_ptr<SoftRobots>& soft_robots,
                         const std::shared_ptr<ForceContainer>& forces,
                         std::shared_ptr<BaseLogger>& logger, SimParams& sim_params,
                         RenderParams& render_params) {

    sim_params.dt = 5e-3;
    sim_params.sim_time = 1.0;
    sim_params.dtol = 1e-3;
    sim_params.enable_2d_sim = true;
    sim_params.adaptive_time_stepping = 10;
    sim_params.integrator = IMPLICIT_MIDPOINT;

    render_params.cmd_line_per = 0;
    render_params.renderer = HEADLESS;

    // Create cylinder
    int n = 26;
    double radius = 0.025;
    double young_mod = 1e5;
    double density = 509.2985;
    double poisson = 0.5;
    double mu = 0.4;
    soft_robots->addLimb(Vec3(0.0, 0.00, 0.025), Vec3(1.0, 0.00, 0.025), n, density, radius,
                         young_mod, poisson, mu);

    // Add gravity
    Vec3 gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(std::make_shared<GravityForce>(soft_robots, gravity_vec));

    // Add floor contact
    double delta = 5e-4;
    double nu = 1e-3;
    double floor_z = 0.0;
    forces->addForce(std::make_shared<FloorContactForce>(soft_robots, delta, nu, floor_z));

    // Create an external constant uniform force
    auto uniform_force = std::make_shared<UniformConstantForce>(soft_robots);

    // Define a force value from script input
    if (argc < 2) {
        throw std::runtime_error("Need to supply a force as input.");
    }
    Vec3 force = Vec3::Zero();
    std::string force_value = argv[1];
    force(0) = stod(force_value);  // apply along x-direction

    // Add uniform constant force to the cylinder
    uniform_force->addForceToLimb(0, force);
    forces->addForce(uniform_force);

    // Set logger to record velocities
    std::string logfile_base = "log_files/friction_case";
    int logging_period = 1;
    logger = std::make_shared<VelocityLogger>(logfile_base, force_value, logging_output_file,
                                              logging_period);
}
