#include "explicitTimeStepper.h"


explicitTimeStepper::explicitTimeStepper(const shared_ptr<softRobots>& soft_robots,
                                         const shared_ptr<forceContainer>& forces,
                                         const simParams& sim_params) :
                                         baseTimeStepper(soft_robots, forces, sim_params)
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
void explicitTimeStepper::integrator() {}
void explicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx) {}
void explicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) {}
