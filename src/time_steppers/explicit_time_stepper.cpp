#include "explicit_time_stepper.h"

ExplicitTimeStepper::ExplicitTimeStepper(const shared_ptr<SoftRobots>& soft_robots,
                                         const shared_ptr<ForceContainer>& forces,
                                         const SimParams& sim_params)
    : BaseTimeStepper(soft_robots, forces, sim_params) {
    constructInverseMassVector();
}

ExplicitTimeStepper::~ExplicitTimeStepper() = default;

void ExplicitTimeStepper::prepSystemForIteration() {
    BaseTimeStepper::prepSystemForIteration();
    BaseTimeStepper::setZero();
}

void ExplicitTimeStepper::constructInverseMassVector() {
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
        inverse_masses[j_limb][4 * j_node] = inv_mass;
        inverse_masses[j_limb][4 * j_node + 1] = inv_mass;
        inverse_masses[j_limb][4 * j_node + 2] = inv_mass;
    }
}

// We simply define these to make sure derived classes are not abstract classes
// Perhaps a better way to design this later
void ExplicitTimeStepper::integrator() {
}

void ExplicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx) {
}

void ExplicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) {
}
