#ifndef FORWARD_EULER_H
#define FORWARD_EULER_H

#include "explicitTimeStepper.h"

class forwardEuler : public explicitTimeStepper
{
public:
    forwardEuler(const shared_ptr<softRobots>& soft_robots,
                 const shared_ptr<forceContainer>& forces,
                 const simParams& sim_params);
    ~forwardEuler() override;

    double stepForwardInTime() override;
    void updateSystemForNextTimeStep() override;
};


#endif
