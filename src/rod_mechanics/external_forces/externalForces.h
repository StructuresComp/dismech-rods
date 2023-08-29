#ifndef EXTERNAL_FORCES_H
#define EXTERNAL_FORCES_H

#include "eigenIncludes.h"
#include "externalGravityForce.h"
#include "dampingForce.h"
#include "floorContactForce.h"

// TODO: add non-floor contact functionality via IMC

class externalForces
{
public:
    externalForces(shared_ptr<externalGravityForce> m_gravity_force, shared_ptr<dampingForce> m_damping_force,
                   shared_ptr<floorContactForce> m_floor_contact_force);
    ~externalForces();

    void computeForces(double dt);
    void computeForcesAndJacobian(double dt);

    void setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper);

    shared_ptr<floorContactForce> floor_contact_force; // to log min_dist for now

private:
    shared_ptr<externalGravityForce> gravity_force;
    shared_ptr<dampingForce> damping_force;
};


#endif