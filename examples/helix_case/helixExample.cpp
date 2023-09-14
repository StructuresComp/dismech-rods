#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Helix Under Gravity Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Read vertices describing helical shape
    vector<Vector3d> vertices;
    load_txt<Vector3d>("examples/helix_case/helix_configuration.txt", vertices);

    // Create the helix using custom config initializer
    soft_robots->addLimb(vertices, density, rodRadius, youngM, shearM);

    // Fix top end
    soft_robots->lockEdge(0, 0);

    // Set logger to record nodes
    string logfile_base = input_data.GetStringOpt("logfileBase");
    int logging_period = input_data.GetIntOpt("loggingPeriod");
    logger = make_shared<rodNodeLogger>(logfile_base, convert_float_to_scientific_str(youngM),
                                        logging_output_file, logging_period);
}
