#ifndef BASE_FORCE_CONTAINER_H
#define BASE_FORCE_CONTAINER_H

#include "baseForce.h"

class baseForceContainer
{
public:
    baseForceContainer();
    ~baseForceContainer();

    void computeForces(double dt);
    void computeForcesAndJacobian(double dt);

    void setupForceStepperAccess(const shared_ptr<baseTimeStepper>& stepper);


protected:
    vector<shared_ptr<baseForce>> forces;
};

#endif
