#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Spider on Incline Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Create the limbs
    soft_robots->addLimb(Vector3d(0, 0, 0.20), Vector3d(0, 0.00, 0.10), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0.10, 0, 0.10), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0, 0.10, 0.10), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(0, -0.10, 0.10), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0, 0, 0.10), Vector3d(-0.10, 0, 0.10), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.10, 0, 0.10), Vector3d(0.10, 0, 0.00), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.0, 0.10, 0.10), Vector3d(0.0, 0.10, 0.00), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.0, -0.10, 0.10), Vector3d(0.0, -0.10, 0.00), 25, density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(-0.10, 0, 0.10), Vector3d(-0.10, 0, 0.00), 25, density, rodRadius, youngM, shearM);

    // Create joints and connect appropriately
    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 1, 0);
    soft_robots->addToJoint(0, 2, 0);
    soft_robots->addToJoint(0, 3, 0);
    soft_robots->addToJoint(0, 4, 0);
    soft_robots->createJoint(1, -1);
    soft_robots->addToJoint(1, 5, 0);
    soft_robots->createJoint(2, -1);
    soft_robots->addToJoint(2, 6, 0);
    soft_robots->createJoint(3, -1);
    soft_robots->addToJoint(3, 7, 0);
    soft_robots->createJoint(4, -1);
    soft_robots->addToJoint(4, 8, 0);

    // Apply an external force
//    shared_ptr<uniformConstantForce> uniform_force = make_shared<uniformConstantForce>(soft_robots);
//    Vector3d force = Vector3d::Zero();
//    force(2) = stod(argv[2]);
//    uniform_force->add_force_to_limb(1, force);
//    forces.emplace_back(uniform_force);

//    string logfile_base = input_data.GetStringOpt("logfileBase");
//    int logging_period = input_data.GetIntOpt("loggingPeriod");
//    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
