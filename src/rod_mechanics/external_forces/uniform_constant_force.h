#ifndef UNIFORM_CONSTANT_FORCE_H
#define UNIFORM_CONSTANT_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class UniformConstantForce : public BaseForce
{
  public:
    explicit UniformConstantForce(const shared_ptr<SoftRobots>& m_soft_robots);
    ~UniformConstantForce() override;

    void addForceToLimb(int limb_idx, const Vector3d& force);

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    vector<pair<int, Vector3d>> limb_force_pairs;
};

#endif  // UNIFORM_CONSTANT_FORCE_H
