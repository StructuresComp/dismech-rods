#include "dampingForce.h"
#include "../time_steppers/baseTimeStepper.h"

dampingForce::dampingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                           const vector<shared_ptr<elasticJoint>>& m_joints,
                           double m_viscosity) :
                           baseForce(m_limbs, m_joints), viscosity(m_viscosity)
{
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
            if (limb->isEdgeJoint[i]) continue;
            force = -viscosity * (limb->getVertex(i) - limb->getPreVertex(i)) / limb->dt * limb->voronoiLen(i);
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addForce(ind, -force[k], limb_idx); // subtracting external force
            }
        }
        limb_idx++;
    }

    // TODO: Damping force for connections between limb and joint has to be added
    int n1, l1;
    int n2, l2;
    for (const auto& joint : joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i+1; j < joint->ne; j++) {
                force = -viscosity * (joint->x - joint->x0) / joint->dt * joint->voronoi_len(curr_iter);
                for (int k = 0; k < 3; k++) {
                    ind = 4 * joint->joint_node + k;
                    stepper->addForce(ind, -force[k], joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}

void dampingForce::computeJd()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        // Here, we take advantage of the fact that the damping force Jacobian is a diagonal matrix of identical values.
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i]) continue;
            jac = -viscosity * limb->voronoiLen(i) / limb->dt;
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addJacobian(ind, ind, -jac, limb_idx);
            }
        }
        limb_idx++;
    }

    for (const auto& joint : joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i+1; j < joint->ne; j++) {
                jac = -viscosity * joint->voronoi_len(curr_iter) / joint->dt;
                for (int k = 0; k < 3; k++) {
                    ind = 4 * joint->joint_node + k;
                    stepper->addJacobian(ind, ind, -jac, joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}
