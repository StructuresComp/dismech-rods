#include "dampingForce.h"
#include "time_steppers/baseTimeStepper.h"

dampingForce::dampingForce(const shared_ptr<softRobots>& m_soft_robots, double m_viscosity) :
                           baseForce(m_soft_robots), viscosity(m_viscosity)
{
}

dampingForce::~dampingForce()
{
    ;
}

void dampingForce::computeForce(double dt)
{
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i]) continue;
            force = -viscosity * (limb->getVertex(i) - limb->getPreVertex(i)) / dt * limb->voronoiLen(i);
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
    for (const auto& joint : soft_robots->joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i+1; j < joint->ne; j++) {
                force = -viscosity * (joint->x - joint->x0) / dt * joint->voronoi_len(curr_iter);
                for (int k = 0; k < 3; k++) {
                    ind = 4 * joint->joint_node + k;
                    stepper->addForce(ind, -force[k], joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}

void dampingForce::computeForceAndJacobian(double dt)
{
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        // Here, we take advantage of the fact that the damping force Jacobian is a diagonal matrix of identical values.
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i]) continue;
            jac = -viscosity * limb->voronoiLen(i) / dt;
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addJacobian(ind, ind, -jac, limb_idx);
            }
        }
        limb_idx++;
    }

    for (const auto& joint : soft_robots->joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i+1; j < joint->ne; j++) {
                jac = -viscosity * joint->voronoi_len(curr_iter) / dt;
                for (int k = 0; k < 3; k++) {
                    ind = 4 * joint->joint_node + k;
                    stepper->addJacobian(ind, ind, -jac, joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}
