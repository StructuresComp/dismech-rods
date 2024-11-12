#ifndef BASE_FORCE_H
#define BASE_FORCE_H

#include "global_definitions.h"

class SoftRobots;
class BaseTimeStepper;

class BaseForce
{
  public:
    explicit BaseForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    virtual ~BaseForce() = 0;

    virtual void computeForce(double dt) = 0;
    virtual void computeForceAndJacobian(double dt) = 0;

    void setTimeStepper(std::shared_ptr<BaseTimeStepper> m_stepper);

  protected:
    std::shared_ptr<SoftRobots> soft_robots;
    std::shared_ptr<BaseTimeStepper> stepper = nullptr;
};

#endif  // BASE_FORCE_H
