#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "elasticJoint.h"
#include "../time_steppers/baseTimeStepper.h"

class externalGravityForce
{
public:
    externalGravityForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                         const vector<shared_ptr<elasticJoint>>& m_joints,
                         shared_ptr<baseTimeStepper> m_stepper, Vector3d m_gVector);
    Vector3d gVector;
    void setGravity();
    ~externalGravityForce();
    void computeFg();
    void computeJg();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    shared_ptr<baseTimeStepper> stepper;
    vector<VectorXd> massGravities;
    VectorXd massGravity;
};

#endif
