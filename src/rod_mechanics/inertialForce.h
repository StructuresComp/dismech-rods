#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "elasticJoint.h"
#include "../time_steppers/baseTimeStepper.h"

class inertialForce
{
public:
    inertialForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                  const vector<shared_ptr<elasticJoint>>& m_joints,
                  shared_ptr<baseTimeStepper> m_stepper);

    ~inertialForce();
    void computeFi();
    void computeJi();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    shared_ptr<baseTimeStepper> stepper;

    int ind1, ind2, mappedInd1, mappedInd2;	
    double f, jac;
    double dt;
};

#endif
