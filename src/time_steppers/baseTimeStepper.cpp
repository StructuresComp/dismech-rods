#include "baseTimeStepper.h"

baseTimeStepper::baseTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs,
                                 const vector<shared_ptr<elasticJoint>>& m_joints,
                                 const vector<shared_ptr<rodController>>& m_controllers,
                                 shared_ptr<elasticStretchingForce> m_stretch_force,
                                 shared_ptr<elasticBendingForce> m_bending_force,
                                 shared_ptr<elasticTwistingForce> m_twisting_force,
                                 shared_ptr<inertialForce> m_inertial_force,
                                 shared_ptr<externalGravityForce> m_gravity_force,
                                 shared_ptr<dampingForce> m_damping_force,
                                 shared_ptr<floorContactForce> m_floor_contact_force,
                                 double m_dt) :
                                 limbs(m_limbs), joints(m_joints), controllers(m_controllers),
                                 stretching_force(m_stretch_force), bending_force(m_bending_force),
                                 twisting_force(m_twisting_force), inertial_force(m_inertial_force),
                                 gravity_force(m_gravity_force), damping_force(m_damping_force),
                                 floor_contact_force(m_floor_contact_force), dt(m_dt),
                                 Force(nullptr, 0), DX(nullptr, 0)

{
    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }

    force = new double[freeDOF]{0};
    new (&Force) Map<VectorXd>(force, freeDOF);

    dx = new double[freeDOF]{0};
    new (&DX) Map<VectorXd>(dx, freeDOF);
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
    delete [] force;
}


void baseTimeStepper::addForce(int ind, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0) // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        force[mappedInd + offset] += p; // subtracting elastic force
    }
}


void baseTimeStepper::setZero()
{
    Force.setZero();
}

void baseTimeStepper::update()
{
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    delete [] force;
    delete [] dx;

    force = new double[freeDOF]{0};
    new (&Force) Map<VectorXd>(force, freeDOF);

    dx = new double[freeDOF]{0};
    new (&DX) Map<VectorXd>(dx, freeDOF);
}


void baseTimeStepper::prepSystemForIteration()
{
    for (const auto& joint : joints) joint->prepLimbs();
    for (const auto& limb : limbs) limb->prepareForIteration();
    for (const auto& joint : joints) joint->prepareForIteration();
}
