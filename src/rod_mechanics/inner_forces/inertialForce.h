#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class inertialForce : public baseForce
{
public:
    inertialForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                  const vector<shared_ptr<elasticJoint>>& m_joints);

    ~inertialForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    double f, jac;
};

#endif
