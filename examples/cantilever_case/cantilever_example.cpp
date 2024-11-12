#include "robot_description.h"

extern std::ofstream logging_output_file;  // defined in main.cpp

/*
 * Dynamic Cantilever Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void getRobotDescription(int argc, char** argv, const std::shared_ptr<SoftRobots>& soft_robots,
                         const std::shared_ptr<ForceContainer>& forces,
                         std::shared_ptr<BaseLogger>& logger, SimParams& sim_params,
                         RenderParams& render_params) {

    sim_params.dt = 5e-2;
    sim_params.sim_time = 100;
    sim_params.dtol = 1e-3;
    sim_params.enable_2d_sim = true;
    sim_params.integrator = IMPLICIT_MIDPOINT;

    render_params.show_mat_frames = true;

    int n = 201;
    double radius = 0.02;
    double young_mod = 1e5;
    double density = 500;
    double poisson = 0.5;

    // Create a beam along the x-y plane
    soft_robots->addLimb(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), n, density, radius, young_mod,
                         poisson);

    // Fix one end
    soft_robots->lockEdge(0, 0);

    // Read initial velocity values
    std::vector<Vec3> velocities;
    loadTxt("examples/cantilever_case/cantilever_init_velocity_n=201.txt", velocities);

    // Apply the velocities
    soft_robots->applyInitialVelocities(0, velocities);

    // Set logger to record nodes
    std::string logfile_base = "log_files/cantilever";
    int logging_period = 1;
    logger = std::make_shared<PositionLogger>(logfile_base, convertFloatToScientificStr(young_mod),
                                              logging_output_file, logging_period);
}
