#include "externalGravityForce.h"

externalGravityForce::externalGravityForce(vector<shared_ptr<elasticRod>> m_limbs,
                                           vector<shared_ptr<Joint>> m_joints,
                                           shared_ptr<timeStepper> m_stepper, Vector3d m_gVector)
{
    limbs = m_limbs;
    joints = m_joints;
    stepper = m_stepper;
    gVector = m_gVector;
    setGravity();
}

externalGravityForce::~externalGravityForce()
{
    ;
}

void externalGravityForce::computeFg()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        massGravity = massGravities[limb_idx];
        for (int i = 0; i < limb->ndof; i++)
        {
            if (limb->isDOFJoint[i]) continue;
            stepper->addForce(i, -massGravity[i], limb_idx); // subtracting gravity force
        }
        limb_idx++;
    }

    // TODO: store these values like above
    double force;
    for (const auto & joint : joints) {
        for (int i = 0; i < 3; i++) {
            force = gVector[i] * joint->mass;
            stepper->addForce(4*joint->joint_node+i, -force, joint->joint_limb);
        }
    }
}

void externalGravityForce::computeJg()
{
    ;
}

void externalGravityForce::setGravity()
{
    for (const auto& limb : limbs) {
        massGravity = VectorXd::Zero(limb->ndof);
        for (int i = 0; i < limb->nv; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                int ind = 4*i + k;
                massGravity[ind] = gVector[k] * limb->massArray[ind];
            }
        }
        massGravities.push_back(massGravity);
    }
}
