#include "base_force.h"

BaseForce::BaseForce(const std::shared_ptr<SoftRobots>& soft_robots) : soft_robots(soft_robots) {
}

BaseForce::~BaseForce() = default;

void BaseForce::setTimeStepper(std::weak_ptr<BaseTimeStepper> stepper) {
    weak_stepper = stepper;
}
