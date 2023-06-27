#include "explicitTimeStepper.h"


explicitTimeStepper::explicitTimeStepper(const vector<shared_ptr<elasticRod>> &m_limbs,
                                         const vector<shared_ptr<elasticJoint>> &m_joints,
                                         const vector<shared_ptr<rodController>> &m_controllers,
                                         shared_ptr<elasticStretchingForce> m_stretch_force,
                                         shared_ptr<elasticBendingForce> m_bending_force,
                                         shared_ptr<elasticTwistingForce> m_twisting_force,
                                         shared_ptr<inertialForce> m_inertial_force,
                                         shared_ptr<externalGravityForce> m_gravity_force,
                                         shared_ptr<dampingForce> m_damping_force,
                                         shared_ptr<floorContactForce> m_floor_contact_force, double m_dt) :
                                         baseTimeStepper(m_limbs, m_joints, m_controllers, m_stretch_force, m_bending_force,
                                                         m_twisting_force, m_inertial_force, m_gravity_force,
                                                         m_damping_force, m_floor_contact_force, m_dt)
{

}

explicitTimeStepper::~explicitTimeStepper() = default;


void explicitTimeStepper::prepSystemForIteration()
{
    baseTimeStepper::prepSystemForIteration();
    baseTimeStepper::setZero();
}

// We simply define these to make sure derived classes are not abstract classes
// Perhaps a better way to design this later
void explicitTimeStepper::initSolver() {}
void explicitTimeStepper::integrator() {}
void explicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx) {}
void explicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) {}
