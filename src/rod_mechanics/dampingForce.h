#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "Joint.h"
#include "timeStepper.h"

class dampingForce
{
public:
    dampingForce(vector<shared_ptr<elasticRod>> m_limbs,
                 vector<shared_ptr<Joint>> m_joints,
                 shared_ptr<timeStepper> m_stepper, double m_viscosity);
    ~dampingForce();
    void computeFd();
    void computeJd();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<Joint>> joints;
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;
    double viscosity;

    Vector3d force;
    int ind, indx, indy;
    double jac;
};

#endif
