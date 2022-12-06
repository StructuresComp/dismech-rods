#ifndef INERTIALFORCE_H
#define INERTIALFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "Joint.h"
#include "timeStepper.h"

class inertialForce
{
public:
    inertialForce(vector<shared_ptr<elasticRod>> m_limbs,
                  vector<shared_ptr<Joint>> m_joints,
                  shared_ptr<timeStepper> m_stepper);

    ~inertialForce();
    void computeFi();
    void computeJi();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<Joint>> joints;
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;

    int ind1, ind2, mappedInd1, mappedInd2;	
    double f, jac;
    double dt;
};

#endif
