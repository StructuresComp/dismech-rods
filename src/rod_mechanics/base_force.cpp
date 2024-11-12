#include "base_force.h"

BaseForce::BaseForce(const std::shared_ptr<SoftRobots>& m_soft_robots)
    : soft_robots(m_soft_robots) {
}

BaseForce::~BaseForce() = default;

void BaseForce::setTimeStepper(std::shared_ptr<BaseTimeStepper> m_stepper) {
    stepper = m_stepper;
}
