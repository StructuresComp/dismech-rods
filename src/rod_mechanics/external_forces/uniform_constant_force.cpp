#include "uniform_constant_force.h"
#include "time_steppers/base_time_stepper.h"

UniformConstantForce::UniformConstantForce(const std::shared_ptr<SoftRobots>& m_soft_robots)
    : BaseForce(m_soft_robots) {
}

UniformConstantForce::~UniformConstantForce() = default;

void UniformConstantForce::addForceToLimb(int limb_idx, const Vec3& force) {
    limb_force_pairs.emplace_back(limb_idx, force);
}

void UniformConstantForce::computeForce(double dt) {
    int limb_idx;
    std::shared_ptr<ElasticRod> limb;
    Vec3 force;
    for (const auto& limb_force_pair : limb_force_pairs) {
        limb_idx = limb_force_pair.first;

        limb = soft_robots->limbs[limb_idx];

        force = limb_force_pair.second / limb->ne;

        for (int i = 1; i < limb->nv - 1; i++) {
            if (limb->isNodeJoint[i])
                continue;
            stepper->addForce(4 * i, force(0), limb_idx);
            stepper->addForce(4 * i + 1, force(1), limb_idx);
            stepper->addForce(4 * i + 2, force(2), limb_idx);
        }
        if (!limb->isNodeJoint[0]) {
            stepper->addForce(0, 0.5 * force(0), limb_idx);
            stepper->addForce(1, 0.5 * force(1), limb_idx);
            stepper->addForce(2, 0.5 * force(2), limb_idx);
        }
        if (!limb->isNodeJoint[limb->nv - 1]) {
            stepper->addForce(4 * (limb->nv - 1), 0.5 * force(0), limb_idx);
            stepper->addForce(4 * (limb->nv - 1) + 1, 0.5 * force(1), limb_idx);
            stepper->addForce(4 * (limb->nv - 1) + 2, 0.5 * force(2), limb_idx);
        }

        // Not the most efficient, but okay for now.
        for (const auto& joint : soft_robots->joints) {
            if (joint->joint_limb == limb_idx) {
                if (joint->joint_node == 0 || joint->joint_node == limb->nv - 1) {
                    stepper->addForce(4 * joint->joint_node, 0.5 * force(0), joint->joint_limb);
                    stepper->addForce(4 * joint->joint_node + 1, 0.5 * force(1), joint->joint_limb);
                    stepper->addForce(4 * joint->joint_node + 2, 0.5 * force(2), joint->joint_limb);
                }
                else {
                    stepper->addForce(4 * joint->joint_node, force(0), joint->joint_limb);
                    stepper->addForce(4 * joint->joint_node + 1, force(1), joint->joint_limb);
                    stepper->addForce(4 * joint->joint_node + 2, force(2), joint->joint_limb);
                }
            }
        }
    }
}

void UniformConstantForce::computeForceAndJacobian(double dt) {
    computeForce(dt);
}
