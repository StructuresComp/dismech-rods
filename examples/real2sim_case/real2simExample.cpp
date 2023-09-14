#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Real2Sim Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Create two appendages
    soft_robots->addLimb(Vector3d(0.0, 0.0, 0.0), Vector3d(0.075, 0.0, 0.0), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.075, 0.0, 0.0), Vector3d(0.15, 0.0, 0.0), 25, density, rodRadius, youngM, shearM);

    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 1, 0);

    // Fix one end
    soft_robots->lockEdge(0, 0);

    // Set logger to record nodes
    string logfile_base = input_data.GetStringOpt("logfileBase");
    int logging_period = input_data.GetIntOpt("loggingPeriod");
    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
