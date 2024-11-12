#ifndef FORWARD_EULER_H
#define FORWARD_EULER_H

#include "explicit_time_stepper.h"

class ForwardEuler : public ExplicitTimeStepper
{
  public:
    ForwardEuler(const std::shared_ptr<SoftRobots>& soft_robots,
                 const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params);
    ~ForwardEuler() override;

    double stepForwardInTime() override;
    void updateSystemForNextTimeStep() override;
};

#endif  // FORWARD_EULER_H
