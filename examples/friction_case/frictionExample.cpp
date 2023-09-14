#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Axial Friction Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Create cylinder
    soft_robots->addLimb(Vector3d(0.0, 0.00, 0.025), Vector3d(1.0, 0.00, 0.025), 51, density, rodRadius, youngM,
                         shearM);

    // Create an external constant uniform force
    shared_ptr<uniformConstantForce> uniform_force = make_shared<uniformConstantForce>(soft_robots);

    // Define a force value from script input
    Vector3d force = Vector3d::Zero();
    string force_value = argv[2];
    force(0) = stod(force_value);  // apply along x-direction

    // Add uniform constant force to the cylinder
    uniform_force->add_force_to_limb(0, force);
    forces.emplace_back(uniform_force);

    // Set logger to record velocities
    string logfile_base = input_data.GetStringOpt("logfileBase");
    int logging_period = input_data.GetIntOpt("loggingPeriod");
    logger = make_shared<velocityLogger>(logfile_base, force_value, logging_output_file, logging_period);
}
