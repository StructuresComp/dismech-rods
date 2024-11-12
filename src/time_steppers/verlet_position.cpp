#include "verlet_position.h"
#include <ctime>

VerletPosition::VerletPosition(const std::shared_ptr<SoftRobots>& soft_robots,
                               const std::shared_ptr<ForceContainer>& forces,
                               const SimParams& sim_params)
    : ExplicitTimeStepper(soft_robots, forces, sim_params) {
}

VerletPosition::~VerletPosition() = default;

double VerletPosition::stepForwardInTime() {
    // First position half step
    // Update q_t+dt/2 = q_t + v_t*dt/2
    for (const auto& limb : limbs) {
        limb->x = limb->x0 + limb->u * 0.5 * dt;
    }

    // Perform collision detection if contact is enabled
    if (forces->cf)
        forces->cf->broadPhaseCollisionDetection();

    // Evaluation of local accelerations
    // compute F(q_t+dt/2)
    // Make sure to leave out inertial force
    prepSystemForIteration();
    forces->computeForces(0.5 * dt);

    // Could perhaps explore a vectorized solution for this later but too
    // complicated for now.
    int counter = 0;
    double acceleration;
    int limb_num = 0;
    for (const auto& limb : limbs) {
        for (int local_counter = 0; local_counter < limb->ndof; local_counter++) {
            if (!limb->isConstrained[local_counter] && limb->isDOFJoint[local_counter] != 1) {
                // Computing accelerations
                // a_t+dt/2 = M^{-1} @ F
                acceleration = inverse_masses[limb_num][local_counter] * -Force[counter];

                // Update velocity u_t+dt = u_t + a_t+dt/2 * dt
                limb->u[local_counter] = limb->u[local_counter] + acceleration * dt;

                // Update position x_t+dt = x_t+dt/2 + u_t+dt * dt / 2
                limb->x[local_counter] = limb->x[local_counter] + limb->u[local_counter] * dt / 2;

                counter++;
            }
        }
        limb_num++;
    }
    updateSystemForNextTimeStep();
    return dt;
}

void VerletPosition::updateSystemForNextTimeStep() {
    for (const auto& controller : controllers) {
        controller->updateTimeStep(dt);
    }
    // Update x0
    for (const auto& limb : limbs) {
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
