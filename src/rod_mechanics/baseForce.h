#ifndef BASEFORCE_H
#define BASEFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "elasticJoint.h"

class baseTimeStepper;

class baseForce
{
public:
    baseForce(const vector<shared_ptr<elasticRod>>& m_limbs,
              const vector<shared_ptr<elasticJoint>>& m_joints);
    virtual ~baseForce() = 0;

    virtual void computeForce(double dt) = 0;
    virtual void computeForceAndJacobian(double dt) = 0;

    void setTimeStepper(shared_ptr<baseTimeStepper> m_stepper);

protected:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    shared_ptr<baseTimeStepper> stepper = nullptr;
};

#endif
