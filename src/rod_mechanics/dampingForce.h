#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class dampingForce
{
public:
    dampingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, double m_viscosity);
    ~dampingForce();
    void computeFd();
    void computeJd();

private:
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;
    double viscosity;

    Vector3d force;
    int ind, indx, indy;
    double jac;
};

#endif
