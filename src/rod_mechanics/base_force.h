#ifndef BASE_FORCE_H
#define BASE_FORCE_H

#include "global_definitions.h"

class SoftRobots;
class BaseTimeStepper;

class BaseForce
{
  public:
    explicit BaseForce(const std::shared_ptr<SoftRobots>& soft_robots);
    virtual ~BaseForce() = 0;

    virtual void computeForce(double dt) = 0;
    virtual void computeForceAndJacobian(double dt) = 0;

    void setTimeStepper(std::weak_ptr<BaseTimeStepper> stepper);

  protected:
    std::shared_ptr<SoftRobots> soft_robots;
    std::weak_ptr<BaseTimeStepper> weak_stepper;
};

#endif  // BASE_FORCE_H
