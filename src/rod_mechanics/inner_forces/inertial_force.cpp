#include "inertial_force.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include "time_steppers/base_time_stepper.h"

InertialForce::InertialForce(const std::shared_ptr<SoftRobots>& m_soft_robots)
    : BaseForce(m_soft_robots) {
}

InertialForce::~InertialForce() {
    ;
}

void InertialForce::computeForce(double dt) {
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i])
                continue;
            f = (limb->mass_array[i] / dt) * ((limb->x[i] - limb->x0[i]) / dt - limb->u[i]);
            stepper->addForce(i, f, limb_idx);
        }
        limb_idx++;
    }

    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            f = (joint->mass / dt) * ((joint->x[i] - joint->x0[i]) / dt - joint->u[i]);
            stepper->addForce(4 * joint->joint_node + i, f, joint->joint_limb);
        }
    }
}

void InertialForce::computeForceAndJacobian(double dt) {
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i])
                continue;
            jac = limb->mass_array(i) / (dt * dt);
            stepper->addJacobian(i, i, jac, limb_idx);
        }
        limb_idx++;
    }

    int ind;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            jac = joint->mass / (dt * dt);
            ind = 4 * joint->joint_node;
            stepper->addJacobian(ind + i, ind + i, jac, joint->joint_limb);
        }
    }
}
