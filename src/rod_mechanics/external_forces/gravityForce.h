#ifndef GRAVITYFORCE_H
#define GRAVITYFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class gravityForce : public baseForce
{
  public:
    gravityForce(const shared_ptr<softRobots>& soft_robots, Vector3d g_vector);
    ~gravityForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    Vector3d g_vector;
    void setGravity();
    vector<VectorXd> mass_gravities;
    VectorXd mass_gravity;
};

#endif
