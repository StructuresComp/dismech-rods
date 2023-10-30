#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Helix Under Gravity Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params) {

    sim_params.dt = 5e-3;
    sim_params.sim_time = 10;
    sim_params.dtol = 1e-3;
    sim_params.render_scale = 5.0;
    sim_params.nis = IMPLICIT_MIDPOINT;

    // Read vertices describing helical shape
    vector<Vector3d> vertices;
    load_txt<Vector3d>("examples/helix_case/helix_configuration.txt", vertices);

    // Create the helix using custom config initializer
    double radius = 5e-3;
    double young_mod = 1e7;
    double density = 1273.52;
    double poisson = 0.5;
    soft_robots->addLimb(vertices, density, radius, young_mod, poisson);

    // Fix top end
    soft_robots->lockEdge(0, 0);

    // Add gravity
    Vector3d gravity_vec(0.0, 0.0, -9.8);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vec));

    // Set logger to record nodes
    string logfile_base = "log_files/helix";
    int logging_period = 1;
    logger = make_shared<rodNodeLogger>(logfile_base, convert_float_to_scientific_str(young_mod),
                                        logging_output_file, logging_period);
}
