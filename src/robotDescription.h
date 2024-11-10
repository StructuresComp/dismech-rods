#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include "rod_mechanics/forceContainer.h"
#include "rod_mechanics/softRobots.h"

// include external forces
#include "rod_mechanics/external_forces/contactForce.h"
#include "rod_mechanics/external_forces/dampingForce.h"
#include "rod_mechanics/external_forces/floorContactForce.h"
#include "rod_mechanics/external_forces/gravityForce.h"
#include "rod_mechanics/external_forces/uniformConstantForce.h"

// include controllers
#include "controllers/activeEntanglementController.h"
#include "controllers/openLoopUniformKappaBarController.h"

// different type of loggers
#include "logging/rodNodeLogger.h"
#include "logging/velocityLogger.h"

#include "utils/utils.h"

void get_robot_description(int argc, char** argv, const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger, simParams& sim_params,
                           renderParams& render_params);

#endif
