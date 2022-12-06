#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "Joint.h"
#include "timeStepper.h"

class externalGravityForce
{
public:
    externalGravityForce(vector<shared_ptr<elasticRod>> m_limbs,
                         vector<shared_ptr<Joint>> m_joints,
                         shared_ptr<timeStepper> m_stepper, Vector3d m_gVector);
    Vector3d gVector;
    void setGravity();
    ~externalGravityForce();
    void computeFg();
    void computeJg();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<Joint>> joints;
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;
    vector<VectorXd> massGravities;
    VectorXd massGravity;
};

#endif
