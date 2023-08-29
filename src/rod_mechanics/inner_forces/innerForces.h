#ifndef INNER_FORCES_H
#define INNER_FORCES_H

#include "eigenIncludes.h"
#include "inertialForce.h"
#include "elasticStretchingForce.h"
#include "elasticBendingForce.h"
#include "elasticTwistingForce.h"


class innerForces
{
public:
    innerForces(shared_ptr<inertialForce> m_inertial_force, shared_ptr<elasticStretchingForce> m_stretching_force,
                shared_ptr<elasticBendingForce> m_bending_force, shared_ptr<elasticTwistingForce> m_twisting_force);
    ~innerForces();

    void computeForces(double dt);
    void computeForcesAndJacobian(double dt);

    void setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper);

private:
    shared_ptr<inertialForce> inertial_force = nullptr;
    shared_ptr<elasticStretchingForce> stretching_force = nullptr;
    shared_ptr<elasticBendingForce> bending_force = nullptr;
    shared_ptr<elasticTwistingForce> twisting_force = nullptr;
};


#endif