#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include <memory>
#include "rod_mechanics/elasticRod.h"
#include "rod_mechanics/elasticJoint.h"
#include "utils/utils.h"

void get_robot_description(vector<shared_ptr<elasticRod>>& limbs, vector<shared_ptr<elasticJoint>>& joints,
                           double density, double rodRadius, double youngM, double shearM);

#endif
