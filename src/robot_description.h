#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

// include external forces
#include "rod_mechanics/external_forces/contact_force.h"
#include "rod_mechanics/external_forces/damping_force.h"
#include "rod_mechanics/external_forces/floor_contact_force.h"
#include "rod_mechanics/external_forces/gravity_force.h"
#include "rod_mechanics/external_forces/uniform_constant_force.h"

// include controllers
#include "controllers/active_entanglement_controller.h"
#include "controllers/open_loop_uniform_kappa_bar_controller.h"

// different type of loggers
#include "logging/position_logger.h"
#include "logging/velocity_logger.h"

#include "utils/utils.h"

void getRobotDescription(int argc, char** argv, const shared_ptr<SoftRobots>& soft_robots,
                         const shared_ptr<ForceContainer>& forces, shared_ptr<BaseLogger>& logger,
                         SimParams& sim_params, RenderParams& render_params);

#endif
