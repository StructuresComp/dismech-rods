#ifndef DAMPING_FORCE_H
#define DAMPING_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class DampingForce : public BaseForce
{
  public:
    DampingForce(const std::shared_ptr<SoftRobots>& soft_robots, double viscosity);
    ~DampingForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    double viscosity;
};

#endif  // DAMPING_FORCE_H
