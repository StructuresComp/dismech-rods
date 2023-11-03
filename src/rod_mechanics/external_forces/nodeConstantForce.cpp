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

        if (limb_idx >= soft_robots->limbs.size()) {
            std::cout << "ERROR: limb index out of range" << std::endl;
            exit(1);
        }
        limb = soft_robots->limbs[limb_idx];

        force = force_item.force;

        if (node_idx == -1) {
            node_idx = limb->nv-1;
        }

        if (node_idx >= limb->nv) {
            std::cout << "ERROR: node index out of range" << std::endl;
            exit(1);
        }
        stepper->addForce(4*node_idx, force(0), limb_idx);
        stepper->addForce(4*node_idx+1, force(1), limb_idx);
        stepper->addForce(4*node_idx+2, force(2), limb_idx);

    }
}

void nodeConstantForce::computeForceAndJacobian(double dt) {
    computeForce(dt);
}