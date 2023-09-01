#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class inertialForce : public baseForce
{
public:
    inertialForce(const shared_ptr<softRobots>& m_soft_robots);
    ~inertialForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    double f, jac;
};

#endif
