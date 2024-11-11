#ifndef VERLET_POSITION_H
#define VERLET_POSITION_H

#include "explicit_time_stepper.h"

class VerletPosition : public ExplicitTimeStepper
{
  public:
    VerletPosition(const shared_ptr<SoftRobots>& soft_robots,
                   const shared_ptr<ForceContainer>& forces, const SimParams& sim_params);
    ~VerletPosition() override;

    double stepForwardInTime() override;
    void updateSystemForNextTimeStep() override;
};

#endif  // VERLET_POSITION_H
