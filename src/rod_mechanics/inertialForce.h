#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "baseForce.h"

class baseTimeStepper;

class inertialForce : public baseForce
{
public:
    inertialForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                  const vector<shared_ptr<elasticJoint>>& m_joints);

    ~inertialForce() override;
    void computeFi();
    void computeJi();

private:
    double f, jac;
    double dt;
};

#endif
