#include "baseController.h"

baseController::baseController(const vector<shared_ptr<elasticRod>>& limbs)
    : limbs(limbs), num_actuators(limbs.size()), current_time(0) {
}

baseController::~baseController() = default;

// but we can also implement the timestepping here, for others to override if
// desired.
void baseController::updateTimeStep(double dt) {
    current_time += dt;
}
