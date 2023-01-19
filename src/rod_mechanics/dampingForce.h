#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "elasticJoint.h"
#include "timeStepper.h"

class dampingForce
{
public:
    dampingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                 const vector<shared_ptr<elasticJoint>>& m_joints,
                 shared_ptr<timeStepper> m_stepper, double m_viscosity);
    ~dampingForce();
    void computeFd();
    void computeJd();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    shared_ptr<timeStepper> stepper;
    double viscosity;

    Vector3d force;
    int ind, indx, indy;
    double jac;
};

#endif
