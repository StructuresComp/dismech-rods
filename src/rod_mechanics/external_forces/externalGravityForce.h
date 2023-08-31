#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class externalGravityForce : public baseForce
{
public:
    externalGravityForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                         const vector<shared_ptr<elasticJoint>>& m_joints,
                         Vector3d m_gVector);
    ~externalGravityForce() override;

    Vector3d gVector;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    void setGravity();
    vector<VectorXd> massGravities;
    VectorXd massGravity;
};

#endif
