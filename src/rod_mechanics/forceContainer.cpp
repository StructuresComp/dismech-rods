#include "forceContainer.h"

forceContainer::forceContainer() : cf(nullptr), ff(nullptr) {
}

forceContainer::forceContainer(const vector<shared_ptr<baseForce>>& m_forces)
    : cf(nullptr), ff(nullptr) {
    for (const auto& force : m_forces) {
        addForce(force);
    }
}

forceContainer::~forceContainer() = default;

void forceContainer::setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper) {
    for (const auto& force : forces)
        force->setTimeStepper(stepper);
}

void forceContainer::computeForces(double dt) {
    for (const auto& force : forces)
        force->computeForce(dt);
}

void forceContainer::computeForcesAndJacobian(double dt) {
    for (const auto& force : forces)
        force->computeForceAndJacobian(dt);
}

void forceContainer::addForce(const shared_ptr<baseForce>& force) {
    if (cf == nullptr) {
        cf = dynamic_pointer_cast<contactForce>(force);
    }
    if (ff == nullptr) {
        ff = dynamic_pointer_cast<floorContactForce>(force);
    }
    forces.emplace_back(force);
}
