#include "robotDescription.h"


/*
 * Define your soft robot structure in the function below.
 * World will call it to populate its limbs and joints accordingly.
 * Some example structures have been provided, which can played around with.
 */

void get_robot_description(vector<shared_ptr<elasticRod>>& limbs, vector<shared_ptr<elasticJoint>>& joints,
                           double density, double rodRadius, double deltaTime, double youngM, double shearM) {
    // /* Horton case */
    limbs.push_back(make_shared<elasticRod>(0, Vector3d(-0.10, 0.00, 0.00), Vector3d(-0.10, 0.00, 0.10), 15,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(1, Vector3d(0.00, 0.00, 0.00), Vector3d(0.00, 0.00, 0.10), 15,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(2, Vector3d(0.10, 0.00, 0.00), Vector3d(0.10, 0.00, 0.10), 15,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(3, Vector3d(-0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), 15,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(4, Vector3d(0.10, 0.00, 0.10), Vector3d(0.00, 0.00, 0.10), 15,
                                            density, rodRadius, deltaTime, youngM, shearM));
    joints.push_back(make_shared<elasticJoint>(14, 0, limbs));
    joints[0]->addToJoint(0, 3);
    joints.push_back(make_shared<elasticJoint>(14, 1, limbs));
    joints[1]->addToJoint(14, 3);
    joints[1]->addToJoint(14, 4);
    joints.push_back(make_shared<elasticJoint>(14, 2, limbs));
    joints[2]->addToJoint(0, 4);
    
    /* Spider case */
    // limbs.push_back(make_shared<elasticRod>(0, Vector3d(0, 0, 0.10), Vector3d(0, 0.00, 0.05), 15,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(1, Vector3d(0, 0, 0.05), Vector3d(0.05, 0, 0.05), 15,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(2, Vector3d(0, 0, 0.05), Vector3d(0, 0.05, 0.05), 15,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(3, Vector3d(0, 0, 0.05), Vector3d(0, -0.05, 0.05), 15,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(4, Vector3d(0, 0, 0.05), Vector3d(-0.05, 0, 0.05), 15,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(5, Vector3d(0.05, 0, 0.05), Vector3d(0.05, 0, 0.00), 10,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(6, Vector3d(0.0, 0.05, 0.05), Vector3d(0.0, 0.05, 0.00), 10,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(7, Vector3d(0.0, -0.05, 0.05), Vector3d(0.0, -0.05, 0.00), 10,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // limbs.push_back(make_shared<elasticRod>(8, Vector3d(-0.05, 0, 0.05), Vector3d(-0.05, 0, 0.00), 10,
    //                                         density, rodRadius, deltaTime, youngM, shearM));
    // joints.push_back(make_shared<elasticJoint>(14, 0, limbs));
    // joints[0]->addToJoint(0, 1);
    // joints[0]->addToJoint(0, 2);
    // joints[0]->addToJoint(0, 3);
    // joints[0]->addToJoint(0, 4);
    // joints.push_back(make_shared<elasticJoint>(14, 1, limbs));
    // joints[1]->addToJoint(0, 5);
    // joints.push_back(make_shared<elasticJoint>(14, 2, limbs));
    // joints[2]->addToJoint(0, 6);
    // joints.push_back(make_shared<elasticJoint>(14, 3, limbs));
    // joints[3]->addToJoint(0, 7);
    // joints.push_back(make_shared<elasticJoint>(14, 4, limbs));
    // joints[4]->addToJoint(0, 8);

    /* Random grid case */
//    limbs.push_back(make_shared<elasticRod>(0, Vector3d(0, 0, 0), Vector3d(0, 0.00, -0.05), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(1, Vector3d(0, 0, -0.04), Vector3d(0, 0.03, -0.04), 4,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(2, Vector3d(0, 0, -0.05), Vector3d(0.05, 0, -0.05), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(3, Vector3d(0.03, 0, -0.05), Vector3d(0.03, 0.05, -0.05), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    joints.push_back(make_shared<elasticJoint>(4, 0, limbs));
//    joints[0]->addToJoint(0, 1);
//    joints.push_back(make_shared<elasticJoint>(5, 0, limbs));
//    joints[1]->addToJoint(0, 2);
//    joints.push_back(make_shared<elasticJoint>(3, 2, limbs));
//    joints[2]->addToJoint(0, 3);

    /* Chandelier case */
//    limbs.push_back(make_shared<elasticRod>(0, Vector3d(0, 0, 0), Vector3d(0, 0.00, -0.05), 5,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(1, Vector3d(0, 0, -0.05), Vector3d(0.05, 0, -0.05), 8,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(2, Vector3d(0, 0, -0.05), Vector3d(0, 0.05, -0.05), 8,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(3, Vector3d(0, 0, -0.05), Vector3d(0, -0.05, -0.05), 8,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(4, Vector3d(0, 0, -0.05), Vector3d(-0.05, 0, -0.05), 8,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    joints[0]->addToJoint(0, 1);
//    joints[0]->addToJoint(0, 2);
//    joints[0]->addToJoint(0, 3);
//    joints[0]->addToJoint(0, 4);

    /* Closed square case */
//    limbs.push_back(make_shared<elasticRod>(0, Vector3d(0, 0, 0), Vector3d(0, 0.00, -0.05), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(1, Vector3d(0, 0, -0.05), Vector3d(0.05, 0, -0.05), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(2, Vector3d(0.05, 0, -0.05), Vector3d(0.05, 0.0, 0), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(3, Vector3d(0.05, 0, 0), Vector3d(0.0, 0.0, 0), 6,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    joints.push_back(make_shared<elasticJoint>(5, 0, limbs));
//    joints[0]->addToJoint(0, 1);
//    joints.push_back(make_shared<elasticJoint>(5, 1, limbs));
//    joints[1]->addToJoint(0, 2);
//    joints.push_back(make_shared<elasticJoint>(5, 2, limbs));
//    joints[2]->addToJoint(0, 3);
//    joints.push_back(make_shared<elasticJoint>(0, 0, limbs));
//    joints[3]->addToJoint(5, 3);
}
