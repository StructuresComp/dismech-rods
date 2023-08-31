#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include <memory>
#include "rod_mechanics/elasticRod.h"
#include "rod_mechanics/elasticJoint.h"
#include "rod_mechanics/external_forces/uniformConstantForce.h"
#include "utils/utils.h"

void get_robot_description(int argc, char** argv,
                           vector<shared_ptr<elasticRod>>& limbs,
                           vector<shared_ptr<elasticJoint>>& joints,
                           shared_ptr<uniformConstantForce>& uniform_force,
                           double density, double rodRadius,
                           double youngM, double shearM);

#endif
