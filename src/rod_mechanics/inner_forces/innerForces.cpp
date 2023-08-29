#include "innerForces.h"

#include <utility>


innerForces::innerForces(shared_ptr<inertialForce> m_inertial_force,
                         shared_ptr<elasticStretchingForce> m_stretching_force,
                         shared_ptr<elasticBendingForce> m_bending_force,
                         shared_ptr<elasticTwistingForce> m_twisting_force) :
                         inertial_force(std::move(m_inertial_force)), stretching_force(std::move(m_stretching_force)),
                         bending_force(std::move(m_bending_force)), twisting_force(std::move(m_twisting_force))  {
}


innerForces::~innerForces() = default;


void innerForces::setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper) {
    if (inertial_force) inertial_force->setTimeStepper(stepper);
    stretching_force->setTimeStepper(stepper);
    bending_force->setTimeStepper(stepper);
    twisting_force->setTimeStepper(stepper);
}


void innerForces::computeForces(double dt) {
    if (inertial_force) inertial_force->computeFi(dt);
    stretching_force->computeFs();
    bending_force->computeFb();
    twisting_force->computeFt();
}


void innerForces::computeForcesAndJacobian(double dt) {
    if (inertial_force) {
        inertial_force->computeFi(dt);
        inertial_force->computeJi(dt);
    }
    stretching_force->computeFs();
    stretching_force->computeJs();

    bending_force->computeFb();
    bending_force->computeJb();

    twisting_force->computeFt();
    twisting_force->computeJt();
}

