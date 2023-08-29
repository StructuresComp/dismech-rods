#include "explicitTimeStepper.h"


explicitTimeStepper::explicitTimeStepper(const vector<shared_ptr<elasticRod>> &m_limbs,
                                         const vector<shared_ptr<elasticJoint>> &m_joints,
                                         const vector<shared_ptr<rodController>> &m_controllers,
                                         const shared_ptr<innerForces>& m_inner_forces,
                                         const shared_ptr<externalForces>& m_external_forces, double m_dt) :
                                         baseTimeStepper(m_limbs, m_joints, m_controllers, m_inner_forces,
                                                         m_external_forces, m_dt)
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
