#include "baseTimeStepper.h"
#include <utility>

baseTimeStepper::baseTimeStepper(const shared_ptr<softRobots>& m_soft_robots,
                                 shared_ptr<innerForces> m_inner_forces,
                                 shared_ptr<externalForces> m_external_forces,
                                 double m_dt) :
                                 limbs(m_soft_robots->limbs), joints(m_soft_robots->joints),
                                 controllers(m_soft_robots->controllers),
                                 inner_forces(std::move(m_inner_forces)),
                                 external_forces(std::move(m_external_forces)),
                                 dt(m_dt), Force(nullptr, 0), DX(nullptr, 0)

{
    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }

    force = new double[freeDOF]{0};
    new (&Force) Map<VectorXd>(force, freeDOF);

    dx = new double[freeDOF]{0};
    new (&DX) Map<VectorXd>(dx, freeDOF);
}


void baseTimeStepper::setupForceStepperAccess() {
    inner_forces->setupForceStepperAccess(shared_from_this());
    external_forces->setupForceStepperAccess(shared_from_this());
}



baseTimeStepper::~baseTimeStepper()
{
    delete [] dx;
    delete [] force;
}


void baseTimeStepper::addForce(int ind, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0) // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        force[mappedInd + offset] += p; // subtracting elastic force
    }
}


void baseTimeStepper::setZero()
{
    Force.setZero();
}

void baseTimeStepper::update()
{
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    delete [] force;
    delete [] dx;

    force = new double[freeDOF]{0};
    new (&Force) Map<VectorXd>(force, freeDOF);

    dx = new double[freeDOF]{0};
    new (&DX) Map<VectorXd>(dx, freeDOF);
}


void baseTimeStepper::prepSystemForIteration()
{
    for (const auto& joint : joints) joint->prepLimbs();
    for (const auto& limb : limbs) limb->prepareForIteration();
    for (const auto& joint : joints) joint->prepareForIteration();
}
