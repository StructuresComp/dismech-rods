#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class GravityForce : public BaseForce
{
  public:
    GravityForce(const shared_ptr<SoftRobots>& soft_robots, Vector3d g_vector);
    ~GravityForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    void setGravity();

    Vector3d g_vector;
    vector<VectorXd> mass_gravities;
    VectorXd mass_gravity;
};

#endif  // GRAVITY_FORCE_H
