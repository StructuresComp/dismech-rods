#include "baseForceContainer.h"

baseForceContainer::baseForceContainer() = default;

baseForceContainer::~baseForceContainer() = default;

void baseForceContainer::setupForceStepperAccess(const shared_ptr<baseTimeStepper> &stepper) {
    for (const auto& force : forces)
        force->setTimeStepper(stepper);
}


void baseForceContainer::computeForces(double dt) {
    for (const auto& force : forces)
        force->computeForce(dt);
}


void baseForceContainer::computeForcesAndJacobian(double dt) {
    for (const auto& force : forces)
        force->computeForceAndJacobian(dt);
}
