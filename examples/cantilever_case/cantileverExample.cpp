#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Dynamic Cantilever Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Create a beam along the x-y plane
    soft_robots->addLimb(Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), 201, density, rodRadius, youngM, shearM);

    // Fix one end
    soft_robots->lockEdge(0, 0);

    // Read initial velocity values
    vector<Vector3d> velocities;
    load_txt("examples/cantilever_case/cantilever_init_velocity_n=201.txt", velocities);

    // Apply the velocities
    soft_robots->applyInitialVelocities(0, velocities);

    // Set logger to record nodes
    string logfile_base = input_data.GetStringOpt("logfileBase");
    int logging_period = input_data.GetIntOpt("loggingPeriod");
    logger = make_shared<rodNodeLogger>(logfile_base, convert_float_to_scientific_str(youngM),
                                        logging_output_file, logging_period);
}
