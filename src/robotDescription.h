#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include <memory>
#include "rod_mechanics/elasticRod.h"
#include "rod_mechanics/elasticJoint.h"

void get_robot_description(vector<shared_ptr<elasticRod>>& limbs, vector<shared_ptr<elasticJoint>>& joints,
                           double density, double rodRadius, double deltaTime, double youngM, double shearM);

#endif
