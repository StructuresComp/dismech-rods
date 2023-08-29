#include "externalForces.h"


externalForces::externalForces(shared_ptr<externalGravityForce> m_gravity_force,
                               shared_ptr<dampingForce> m_damping_force,
                               shared_ptr<floorContactForce> m_floor_contact_force) :
                               gravity_force(std::move(m_gravity_force)), damping_force(std::move(m_damping_force)),
                               floor_contact_force(std::move(m_floor_contact_force)) {
}


externalForces::~externalForces() = default;


void externalForces::setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper) {

    if (gravity_force) gravity_force->setTimeStepper(stepper);
    if (damping_force) damping_force->setTimeStepper(stepper);
    if (floor_contact_force) floor_contact_force->setTimeStepper(stepper);
}


void externalForces::computeForces(double dt) {
    if (gravity_force) gravity_force->computeFg();
    if (damping_force) damping_force->computeFd(dt);
    if (floor_contact_force) floor_contact_force->computeFf(dt);
}


void externalForces::computeForcesAndJacobian(double dt) {
    if (gravity_force) {
        gravity_force->computeFg();
        gravity_force->computeJg();
    }

    if (damping_force) {
        damping_force->computeFd(dt);
        damping_force->computeJd(dt);
    }

    if (floor_contact_force) floor_contact_force->computeFfJf(dt);
}
