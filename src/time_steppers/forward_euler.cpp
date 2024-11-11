#include "forward_euler.h"

ForwardEuler::ForwardEuler(const shared_ptr<SoftRobots>& soft_robots,
                           const shared_ptr<ForceContainer>& forces, const SimParams& sim_params)
    : ExplicitTimeStepper(soft_robots, forces, sim_params) {
}

ForwardEuler::~ForwardEuler() = default;

double ForwardEuler::stepForwardInTime() {
    // Perform collision detection if contact is enabled
    if (forces->cf)
        forces->cf->broadPhaseCollisionDetection();

    // Compute forces using current x and u
    prepSystemForIteration();
    forces->computeForces(dt);

    // Could perhaps explore a vectorized solution for this later but too
    // complicated for now.
    int counter = 0;
    double acceleration;
    int limb_num = 0;
    for (const auto& limb : limbs) {
        for (int local_counter = 0; local_counter < limb->ndof; local_counter++) {
            if (!limb->isConstrained[local_counter] && limb->isDOFJoint[local_counter] != 1) {
                // Computing accelerations
                // a_t = M^{-1} @ F
                acceleration = inverse_masses[limb_num][local_counter] * -Force[counter];

                // Position update
                // q_t+dt = ((f * dt / m) + u_t) * dt + x0
                limb->x[local_counter] =
                    ((acceleration * dt) + limb->u[local_counter]) * dt + limb->x0[local_counter];

                counter++;
            }
        }
        limb_num++;
    }

    updateSystemForNextTimeStep();
    return dt;
}

void ForwardEuler::updateSystemForNextTimeStep() {
    for (const auto& controller : controllers) {
        controller->updateTimeStep(dt);
    }
    // Update x0 and u
    for (const auto& limb : limbs) {
        limb->u = (limb->x - limb->x0) / dt;
        limb->x0 = limb->x;
    }
    prepSystemForIteration();
    for (const auto& limb : limbs) {
        // Update reference directors
        limb->d1_old = limb->d1;
        limb->d2_old = limb->d2;

        // Update necessary info for time parallel
        limb->tangent_old = limb->tangent;
        limb->ref_twist_old = limb->ref_twist;
    }

    // Do the same updates as above for joints
    for (const auto& joint : joints) {
        joint->d1_old = joint->d1;
        joint->d2_old = joint->d2;
        joint->tangents_old = joint->tangents;
        joint->ref_twist_old = joint->ref_twist;
    }
}
