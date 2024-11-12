#include "gravity_force.h"
#include "time_steppers/base_time_stepper.h"

GravityForce::GravityForce(const std::shared_ptr<SoftRobots>& soft_robots, Vec3 g_vector)
    : BaseForce(soft_robots), g_vector(g_vector) {
    setGravity();
}

GravityForce::~GravityForce() = default;

void GravityForce::computeForce(double dt) {
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        mass_gravity = mass_gravities[limb_idx];
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i])
                continue;
            stepper->addForce(i, -mass_gravity[i],
                              limb_idx);  // subtracting gravity force
        }
        limb_idx++;
    }

    // TODO: store these values like above
    double force;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            force = g_vector[i] * joint->mass;
            stepper->addForce(4 * joint->joint_node + i, -force, joint->joint_limb);
        }
    }
}

void GravityForce::computeForceAndJacobian(double dt) {
    computeForce(dt);
}

void GravityForce::setGravity() {
    for (const auto& limb : soft_robots->limbs) {
        mass_gravity = VecX::Zero(limb->ndof);
        for (int i = 0; i < limb->nv; i++) {
            for (int k = 0; k < 3; k++) {
                int ind = 4 * i + k;
                mass_gravity[ind] = g_vector[k] * limb->mass_array[ind];
            }
        }
        mass_gravities.push_back(mass_gravity);
    }
}
