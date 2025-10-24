#include "base_time_stepper.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

BaseTimeStepper::BaseTimeStepper(const std::shared_ptr<SoftRobots>& soft_robots,
                                 const std::shared_ptr<ForceContainer>& forces,
                                 const SimParams& sim_params)
    : limbs(soft_robots->limbs), joints(soft_robots->joints), controllers(soft_robots->controllers),
      forces(forces), dt(sim_params.dt), Force(nullptr, 0), DX(nullptr, 0) {
    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }

    force = std::vector<double>(freeDOF, 0);
    new (&Force) Eigen::Map<VecX>(force.data(), freeDOF);

    dx = std::vector<double>(freeDOF, 0);
    new (&DX) Eigen::Map<VecX>(dx.data(), freeDOF);
}

void BaseTimeStepper::initStepper() {
    forces->setupForceStepperAccess(shared_from_this());
}

BaseTimeStepper::~BaseTimeStepper() = default;

void BaseTimeStepper::addForce(int ind, double p, int limb_idx) {
    std::shared_ptr<ElasticRod> limb = limbs[limb_idx];

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

    force.clear();
    force.resize(freeDOF, 0);
    new (&Force) Eigen::Map<VecX>(force.data(), freeDOF);

    dx.clear();
    dx.resize(freeDOF, 0);
    new (&DX) Eigen::Map<VecX>(dx.data(), freeDOF);
}

void BaseTimeStepper::prepSystemForIteration() {
    for (const auto& joint : joints)
        joint->prepLimbs();
    for (const auto& limb : limbs)
        limb->prepareForIteration();
    for (const auto& joint : joints)
        joint->prepareForIteration();
}
