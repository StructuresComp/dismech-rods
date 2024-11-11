#include "base_controller.h"

BaseController::BaseController(const vector<shared_ptr<ElasticRod>>& limbs)
    : limbs(limbs), num_actuators(limbs.size()), current_time(0) {
}

BaseController::~BaseController() = default;

// but we can also implement the timestepping here, for others to override if
// desired.
void BaseController::updateTimeStep(double dt) {
    current_time += dt;
}
