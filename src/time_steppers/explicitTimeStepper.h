#ifndef EXPLICITTIMESTEPPER_H
#define EXPLICITTIMESTEPPER_H

#include "baseTimeStepper.h"

class explicitTimeStepper : public baseTimeStepper
{
public:
    explicitTimeStepper(const shared_ptr<softRobots>& m_soft_robots,
                        const shared_ptr<innerForces>& m_inner_forces,
                        const shared_ptr<externalForces>& m_external_forces,
                        double m_dt);
    ~explicitTimeStepper() override;

    void prepSystemForIteration() override;
    void initSolver() override;
    void integrator() override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) override;

};


#endif
