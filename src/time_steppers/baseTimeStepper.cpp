#include "baseTimeStepper.h"

baseTimeStepper::baseTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs)
{
    limbs = m_limbs;

    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    totalForce = new double[freeDOF];
    dx = new double[freeDOF];
    DX = VectorXd::Zero(freeDOF);
}

baseTimeStepper::~baseTimeStepper()
{
    delete [] dx;
    delete [] totalForce;
}

double* baseTimeStepper::getForce()
{
    return totalForce;
}


void baseTimeStepper::addForce(int ind, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0) // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        totalForce[mappedInd + offset] += p; // subtracting elastic force
        Force[mappedInd + offset] += p;
    }
}


void baseTimeStepper::setZero()
{
    for (int i=0; i < freeDOF; i++)
        totalForce[i] = 0;
    Force = VectorXd::Zero(freeDOF);
}

void baseTimeStepper::update()
{
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    delete [] totalForce;
    delete [] dx;

    totalForce = new double[freeDOF];
    dx = new double[freeDOF];
    DX = VectorXd::Zero(freeDOF);
    Force = VectorXd::Zero(freeDOF);
    setZero();
}

