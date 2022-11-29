#include "inertialForce.h"

inertialForce::inertialForce(vector<shared_ptr<elasticRod>> m_limbs, shared_ptr<timeStepper> m_stepper)
{
    limbs = m_limbs;
    stepper = m_stepper;
}

inertialForce::~inertialForce()
{
    ;
}

void inertialForce::computeFi()
{
    // TODOder: we should not need to compute this at every iteration.
    // We should compute and store it in iteration 1 and then reuse it.
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        for (int i=0; i < limb->ndof; i++)
        {
            f = (limb->massArray[i] / limb->dt) * ((limb->x[i] - limb->x0[i]) / limb->dt - limb->u[i]);
            stepper->addForce(i, f, limb_idx);
        }
        limb_idx++;
    }
}

void inertialForce::computeJi()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            jac = limb->massArray(i) / (limb->dt * limb->dt);
            stepper->addJacobian(i, i, jac, limb_idx);
        }
        limb_idx++;
    }
}
