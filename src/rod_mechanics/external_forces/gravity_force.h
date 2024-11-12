#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class GravityForce : public BaseForce
{
  public:
    GravityForce(const std::shared_ptr<SoftRobots>& soft_robots, Vec3 g_vector);
    ~GravityForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    void setGravity();

    Vec3 g_vector;
    std::vector<VecX> mass_gravities;
    VecX mass_gravity;
};

#endif  // GRAVITY_FORCE_H
