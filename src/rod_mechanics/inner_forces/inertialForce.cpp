#include "inertialForce.h"
#include "time_steppers/baseTimeStepper.h"

inertialForce::inertialForce(const shared_ptr<softRobots>& m_soft_robots) :
                             baseForce(m_soft_robots)
{
}

inertialForce::~inertialForce()
{
    ;
}

void inertialForce::computeForce(double dt)
{
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i=0; i < limb->ndof; i++)
        {
            if (limb->isDOFJoint[i]) continue;
            f = (limb->massArray[i] / dt) * ((limb->x[i] - limb->x0[i]) / dt - limb->u[i]);
            stepper->addForce(i, f, limb_idx);
        }
        limb_idx++;
    }

    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            f = (joint->mass / dt) * ((joint->x[i] - joint->x0[i]) / dt - joint->u[i]);
            stepper->addForce(4*joint->joint_node+i, f,  joint->joint_limb);
        }
    }
}

void inertialForce::computeForceAndJacobian(double dt)
{
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i]) continue;
            jac = limb->massArray(i) / (dt * dt);
            stepper->addJacobian(i, i, jac, limb_idx);
        }
        limb_idx++;
    }

    int ind;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            jac = joint->mass / (dt * dt);
            ind = 4*joint->joint_node;
            stepper->addJacobian(ind+i, ind+i, jac, joint->joint_limb);
        }
    }
}
