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
    void computeFi(double dt);
    void computeJi(double dt);

private:
    double f, jac;
};

#endif
