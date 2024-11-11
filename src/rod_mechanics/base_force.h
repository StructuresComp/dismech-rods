#ifndef BASE_FORCE_H
#define BASE_FORCE_H

#include "global_definitions.h"
#include "soft_robots.h"

class BaseTimeStepper;

class BaseForce
{
  public:
    explicit BaseForce(const shared_ptr<SoftRobots>& m_soft_robots);
    virtual ~BaseForce() = 0;

    virtual void computeForce(double dt) = 0;
    virtual void computeForceAndJacobian(double dt) = 0;

    void setTimeStepper(shared_ptr<BaseTimeStepper> m_stepper);

  protected:
    shared_ptr<SoftRobots> soft_robots;
    shared_ptr<BaseTimeStepper> stepper = nullptr;
};

#endif  // BASE_FORCE_H
