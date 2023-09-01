#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class dampingForce : public baseForce
{
public:
    dampingForce(const shared_ptr<softRobots>& m_soft_robots, double m_viscosity);
    ~dampingForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    double viscosity;

    Vector3d force;
    int ind, indx, indy;
    double jac;
};

#endif
