#ifndef VERLETPOSITION_H
#define VERLETPOSITION_H

#include "explicitTimeStepper.h"

class verletPosition : public explicitTimeStepper
{
  public:
    verletPosition(const shared_ptr<softRobots>& soft_robots,
                   const shared_ptr<forceContainer>& forces, const simParams& sim_params);
    ~verletPosition() override;

    double stepForwardInTime() override;
    void updateSystemForNextTimeStep() override;
};

#endif
