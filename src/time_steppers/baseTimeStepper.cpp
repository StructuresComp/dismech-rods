#include "baseTimeStepper.h"

baseTimeStepper::baseTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs,
                                 const vector<shared_ptr<elasticJoint>>& m_joints,
                                 shared_ptr<elasticStretchingForce> m_stretch_force,
                                 shared_ptr<elasticBendingForce> m_bending_force,
                                 shared_ptr<elasticTwistingForce> m_twisting_force,
                                 shared_ptr<inertialForce> m_inertial_force,
                                 shared_ptr<externalGravityForce> m_gravity_force,
                                 shared_ptr<dampingForce> m_damping_force,
                                 shared_ptr<floorContactForce> m_floor_contact_force,
                                 double m_dt) :
                                 limbs(m_limbs), joints(m_joints), stretching_force(m_stretch_force),
                                 bending_force(m_bending_force), twisting_force(m_twisting_force),
                                 inertial_force(m_inertial_force), gravity_force(m_gravity_force),
                                 damping_force(m_damping_force), floor_contact_force(m_floor_contact_force),
                                 dt(m_dt)

{
    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    totalForce = new double[freeDOF];
    dx = new double[freeDOF];
    DX = VectorXd::Zero(freeDOF);
}


void baseTimeStepper::setupForceStepperAccess() {
    stretching_force->setTimeStepper(shared_from_this());
    bending_force->setTimeStepper(shared_from_this());
    twisting_force->setTimeStepper(shared_from_this());
    inertial_force->setTimeStepper(shared_from_this());
    gravity_force->setTimeStepper(shared_from_this());
    damping_force->setTimeStepper(shared_from_this());
    floor_contact_force->setTimeStepper(shared_from_this());
}



baseTimeStepper::~baseTimeStepper()
{
    delete [] dx;
    delete [] totalForce;
}

double* baseTimeStepper::getForce()
{
    return totalForce;
}


void baseTimeStepper::addForce(int ind, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0) // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        totalForce[mappedInd + offset] += p; // subtracting elastic force
        Force[mappedInd + offset] += p;
    }
}


void baseTimeStepper::setZero()
{
    for (int i=0; i < freeDOF; i++)
        totalForce[i] = 0;
    Force = VectorXd::Zero(freeDOF);
}

void baseTimeStepper::update()
{
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    delete [] totalForce;
    delete [] dx;

    totalForce = new double[freeDOF];
    dx = new double[freeDOF];
    DX = VectorXd::Zero(freeDOF);
    Force = VectorXd::Zero(freeDOF);
    setZero();
}

