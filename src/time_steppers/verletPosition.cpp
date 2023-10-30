#include "verletPosition.h"


verletPosition::verletPosition(const shared_ptr<softRobots>& soft_robots,
                               const shared_ptr<forceContainer>& forces,
                               const simParams& sim_params) :
                               explicitTimeStepper(soft_robots, forces, sim_params)
{
    constructInverseMassVector();
}


verletPosition::~verletPosition() = default;


void verletPosition::constructInverseMassVector() {
    int total_dof = 0;
    for (const auto& limb : limbs) {
        VectorXd curr_inv_masses = VectorXd::Zero(limb->ndof);
        for (int i = 0; i < limb->ndof; i++) {
            curr_inv_masses[i] = 1 / limb->mass_array[i];
        }
        inverse_masses.push_back(curr_inv_masses);
        total_dof += limb->ndof;
    }

    // Replace the masses for the proper ones stored in joints
    for (const auto& joint : joints) {
        int j_node = joint->joint_node;
        int j_limb = joint->joint_limb;
        double inv_mass = 1 / joint->mass;
        inverse_masses[j_limb][4*j_node] = inv_mass;
        inverse_masses[j_limb][4*j_node+1] = inv_mass;
        inverse_masses[j_limb][4*j_node+2] = inv_mass;
    }
}


double verletPosition::stepForwardInTime() {
    // First position half step
    // Update q_t+dt/2 = q_t + v_t*dt/2
    for (const auto& limb : limbs) {
        limb->x = limb->x0 + limb->u * 0.5 * dt;
    }

    // Perform collision detection if contact is enabled
    if (forces->cf) forces->cf->broadPhaseCollisionDetection();

    // Evaluation of local accelerations
    // compute F(q_t+dt/2)
    // Make sure to leave out inertial force
    prepSystemForIteration();
    forces->computeForces(0.5 * dt);

    // Could perhaps explore a vectorized solution for this later but too complicated for now.
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


void verletPosition::updateSystemForNextTimeStep() {
    for (const auto& controller : controllers) {
        controller->updateTimestep(dt);
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

