#include "nodeConstantForce.h"
#include "time_steppers/baseTimeStepper.h"
#include <iostream>

// Applies a constant force to a specific node of a specific LIMB
// to limbs only, not joints
nodeConstantForce::nodeConstantForce(const shared_ptr<softRobots>& m_soft_robots) :
                                           baseForce(m_soft_robots)
{
}

nodeConstantForce::~nodeConstantForce() = default;

void nodeConstantForce::add_force_to_limb_node(int limb_idx, int node_idx, const Vector3d& force) {
    struct limbNodeForceItem force_item;
    force_item.limb_idx = limb_idx;
    force_item.node_idx = node_idx;
    force_item.force = force;
    limb_node_forces.emplace_back(force_item);
}

void nodeConstantForce::computeForce(double dt) {
    int limb_idx;
    int node_idx;
    shared_ptr<elasticRod> limb;
    Vector3d force;
    for (const auto& force_item : limb_node_forces) {
        limb_idx = force_item.limb_idx;
        node_idx = force_item.node_idx;

        if (limb_idx >= soft_robots->limbs.size() || limb_idx < 0) {
            throw runtime_error("ERROR: limb index out of range");
        }
        limb = soft_robots->limbs[limb_idx];

        force = force_item.force;

        if (node_idx == -1) {
            node_idx = limb->nv-1;
        }

        if (node_idx >= limb->nv || node_idx < -1) {
            throw runtime_error("ERROR: node index out of range");
        }
        if (limb->isNodeJoint[node_idx] == 1) { 
            bool force_appended = false;
            // Not the most efficient, but okay for now.
            // This means it is connected to a joint which originated from another limb
            // loop though all joints, and the nodes connected to the joint,
            // and find the joint this node is connected to
            for (const auto& joint : soft_robots->joints) {
                if (force_appended) {
                    break;
                }
                for (const auto& node: joint->replaced_nodes) {
                    if (node.first == node_idx && node.second == limb_idx) {
                        stepper->addForce(4*joint->joint_node, force(0), joint->joint_limb);
                        stepper->addForce(4*joint->joint_node+1, force(1), joint->joint_limb);
                        stepper->addForce(4*joint->joint_node+2, force(2), joint->joint_limb);
                        force_appended = true;
                        break;
                    }
                }
            }
        } else if (limb->isNodeJoint[node_idx] == 2) {
            // This means it is connected to a joint which originated from this limb
            // loop though all joints and find the joint this node is connected to
            for (const auto& joint : soft_robots->joints) {
                if (joint->joint_limb == limb_idx && joint->joint_node == node_idx) {
                    stepper->addForce(4*joint->joint_node, force(0), joint->joint_limb);
                    stepper->addForce(4*joint->joint_node+1, force(1), joint->joint_limb);
                    stepper->addForce(4*joint->joint_node+2, force(2), joint->joint_limb);
                }
            }
        } else {
            stepper->addForce(4*node_idx, force(0), limb_idx);
            stepper->addForce(4*node_idx+1, force(1), limb_idx);
            stepper->addForce(4*node_idx+2, force(2), limb_idx);
        }
    }
}

void nodeConstantForce::computeForceAndJacobian(double dt) {
    computeForce(dt);
}