#include "robotDescription.h"


extern ofstream logging_output_file;  // defined in main.cpp
/*
 * 2D Horton Robot Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */

void get_robot_description(int argc, char** argv, setInput& input_data, const shared_ptr<softRobots>& soft_robots,
                           vector<shared_ptr<baseForce>>& forces, shared_ptr<worldLogger>& logger,
                           double density, double rodRadius, double youngM, double shearM) {

    // Create the limbs
    soft_robots->addLimb(Vector3d(-0.10, 0.00, 0.00), Vector3d(-0.10, 0.00, 0.10), 15,
                         density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.00, 0.00, 0.00), Vector3d(0.00, 0.00, 0.10), 15,
                         density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.10, 0.00, 0.00), Vector3d(0.10, 0.00, 0.10), 15,
                         density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(-0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), 15,
                         density, rodRadius, youngM, shearM);
    soft_robots->addLimb(Vector3d(0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), 15,
                         density, rodRadius, youngM, shearM);

    // Create joints and connect appropriately
    soft_robots->createJoint(0, -1);
    soft_robots->addToJoint(0, 3, 0);
    soft_robots->createJoint(1, -1);
    soft_robots->addToJoint(1, 3, -1);
    soft_robots->addToJoint(1, 4, -1);
    soft_robots->createJoint(2, -1);
    soft_robots->addToJoint(2, 4, 0);
}
