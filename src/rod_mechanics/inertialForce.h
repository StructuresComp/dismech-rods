#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class inertialForce
{
public:
    inertialForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper);
    ~inertialForce();
    void computeFi();
    void computeJi();

private:
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;

    int ind1, ind2, mappedInd1, mappedInd2;	
    double f, jac;
};

#endif
