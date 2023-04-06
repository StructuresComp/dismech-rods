#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "baseForce.h"

class baseTimeStepper;

class dampingForce : public baseForce
{
public:
    dampingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                 const vector<shared_ptr<elasticJoint>>& m_joints,
                 double m_viscosity);
    ~dampingForce() override;
    void computeFd();
    void computeJd();

private:
    double viscosity;

    Vector3d force;
    int ind, indx, indy;
    double jac;
};

#endif
