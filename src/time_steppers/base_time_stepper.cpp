#include "base_time_stepper.h"

BaseTimeStepper::BaseTimeStepper(const shared_ptr<SoftRobots>& soft_robots,
                                 const shared_ptr<ForceContainer>& forces,
                                 const SimParams& sim_params)
    : limbs(soft_robots->limbs), joints(soft_robots->joints), controllers(soft_robots->controllers),
      forces(forces), dt(sim_params.dt), Force(nullptr, 0), DX(nullptr, 0) {
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

void BaseTimeStepper::initStepper() {
    forces->setupForceStepperAccess(shared_from_this());
}

BaseTimeStepper::~BaseTimeStepper() {
    delete[] dx;
    delete[] force;
}

void BaseTimeStepper::addForce(int ind, double p, int limb_idx) {
    shared_ptr<ElasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0)  // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        force[mappedInd + offset] += p;  // subtracting elastic force
    }
}

void BaseTimeStepper::setZero() {
    Force.setZero();
}

void BaseTimeStepper::update() {
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    delete[] force;
    delete[] dx;

    force = new double[freeDOF]{0};
    new (&Force) Map<VectorXd>(force, freeDOF);

    dx = new double[freeDOF]{0};
    new (&DX) Map<VectorXd>(dx, freeDOF);
}

void BaseTimeStepper::prepSystemForIteration() {
    for (const auto& joint : joints)
        joint->prepLimbs();
    for (const auto& limb : limbs)
        limb->prepareForIteration();
    for (const auto& joint : joints)
        joint->prepareForIteration();
}
