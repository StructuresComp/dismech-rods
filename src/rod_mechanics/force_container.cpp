#include "force_container.h"

ForceContainer::ForceContainer() : cf(nullptr), ff(nullptr) {
}

ForceContainer::ForceContainer(const std::vector<std::shared_ptr<BaseForce>>& m_forces)
    : cf(nullptr), ff(nullptr) {
    for (const auto& force : m_forces) {
        addForce(force);
    }
}

ForceContainer::~ForceContainer() = default;

void ForceContainer::setupForceStepperAccess(const std::shared_ptr<BaseTimeStepper>& stepper) {
    for (const auto& force : forces)
        force->setTimeStepper(stepper);
}

void ForceContainer::computeForces(double dt) {
    for (const auto& force : forces)
        force->computeForce(dt);
}

void ForceContainer::computeForcesAndJacobian(double dt) {
    for (const auto& force : forces)
        force->computeForceAndJacobian(dt);
}

void ForceContainer::addForce(const std::shared_ptr<BaseForce>& force) {
    if (cf == nullptr) {
        cf = std::dynamic_pointer_cast<ContactForce>(force);
    }
    if (ff == nullptr) {
        ff = std::dynamic_pointer_cast<FloorContactForce>(force);
    }
    forces.emplace_back(force);
}
