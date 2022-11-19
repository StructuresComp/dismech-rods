#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class externalGravityForce
{
public:
    externalGravityForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, Vector3d m_gVector);
    Vector3d gVector;
    void setGravity();
    ~externalGravityForce();
    void computeFg();
    void computeJg();

private:
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;
    VectorXd massGravity;
};

#endif
