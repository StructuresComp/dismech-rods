#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class externalGravityForce
{
public:
    externalGravityForce(elasticRod &m_rod, timeStepper &m_stepper, Vector3d m_gVector);
    Vector3d gVector;
    void setGravity();
    ~externalGravityForce();
    void computeFg();
    void computeJg();

private:
    elasticRod *rod;
    timeStepper *stepper;
    VectorXd massGravity;
};

#endif
