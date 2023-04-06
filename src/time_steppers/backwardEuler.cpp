#include "backwardEuler.h"

backwardEuler::backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                             shared_ptr<elasticStretchingForce> m_stretch_force,
                             shared_ptr<elasticBendingForce> m_bending_force,
                             shared_ptr<elasticTwistingForce> m_twisting_force,
                             shared_ptr<inertialForce> m_inertial_force,
                             shared_ptr<externalGravityForce> m_gravity_force,
                             shared_ptr<dampingForce> m_damping_force,
                             shared_ptr<floorContactForce> m_floor_contact_force) :
                             implicitTimeStepper(m_limbs, m_stretch_force, m_bending_force,
                                                 m_twisting_force, m_inertial_force, m_gravity_force,
                                                 m_damping_force, m_floor_contact_force)

{
}

backwardEuler::~backwardEuler() = default;


void backwardEuler::integrator()
{
    pardisoSolver();
    // TODO: move the newton's method stuff here
}
