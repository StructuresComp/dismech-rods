#include "baseForce.h"

baseForce::baseForce(const shared_ptr<softRobots>& m_soft_robots) : soft_robots(m_soft_robots)
{
}

baseForce::~baseForce() = default;

void baseForce::setTimeStepper(shared_ptr<baseTimeStepper> m_stepper) {
    stepper = m_stepper;
}
