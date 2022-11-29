#include "dampingForce.h"
#include <iostream>

dampingForce::dampingForce(vector<shared_ptr<elasticRod>> m_limbs, shared_ptr<timeStepper> m_stepper, double m_viscosity)
{
    limbs = m_limbs;
    stepper = m_stepper;
    viscosity = m_viscosity;
}

dampingForce::~dampingForce()
{
    ;
}

void dampingForce::computeFd()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        for (int i = 0; i < limb->ne; i++) {
            force = -viscosity * (limb->getVertex(i) - limb->getPreVertex(i)) / limb->dt * limb->voronoiLen(i);
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addForce(ind, -force[k], limb_idx); // subtracting external force
            }
        }
        limb_idx++;
    }
}

void dampingForce::computeJd()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        // Here, we take advantage of the fact that the damping force Jacobian is a diagonal matrix of identical values.
        for (int i = 0; i < limb->ne; i++) {
            jac = -viscosity * limb->voronoiLen(i) / limb->dt;
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addJacobian(ind, ind, -jac, limb_idx);
            }
        }
        limb_idx++;
    }
}
