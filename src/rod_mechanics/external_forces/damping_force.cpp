#include "damping_force.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include "time_steppers/base_time_stepper.h"

DampingForce::DampingForce(const std::shared_ptr<SoftRobots>& soft_robots, double viscosity)
    : BaseForce(soft_robots), viscosity(viscosity) {
}

DampingForce::~DampingForce() = default;

void DampingForce::computeForce(double dt) {
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i])
                continue;
            Vec3 force = -viscosity * (limb->getVertex(i) - limb->getPreVertex(i)) / dt *
                         limb->voronoi_len(i);
            for (int k = 0; k < 3; k++) {
                int ind = 4 * i + k;
                stepper->addForce(ind, -force[k], limb_idx);  // subtracting external force
            }
        }
        limb_idx++;
    }

    // TODO: Damping force for connections between limb and joint has to be
    // added
    for (const auto& joint : soft_robots->joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i + 1; j < joint->ne; j++) {
                Vec3 force =
                    -viscosity * (joint->x - joint->x0) / dt * joint->voronoi_len(curr_iter);
                for (int k = 0; k < 3; k++) {
                    int ind = 4 * joint->joint_node + k;
                    stepper->addForce(ind, -force[k], joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}

void DampingForce::computeForceAndJacobian(double dt) {
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        // Here, we take advantage of the fact that the damping force Jacobian
        // is a diagonal matrix of identical values.
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i])
                continue;
            double jac = -viscosity * limb->voronoi_len(i) / dt;
            for (int k = 0; k < 3; k++) {
                int ind = 4 * i + k;
                stepper->addJacobian(ind, ind, -jac, limb_idx);
            }
        }
        limb_idx++;
    }

    for (const auto& joint : soft_robots->joints) {
        int curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i + 1; j < joint->ne; j++) {
                double jac = -viscosity * joint->voronoi_len(curr_iter) / dt;
                for (int k = 0; k < 3; k++) {
                    int ind = 4 * joint->joint_node + k;
                    stepper->addJacobian(ind, ind, -jac, joint->joint_limb);
                }
                curr_iter++;
            }
        }
    }
}
