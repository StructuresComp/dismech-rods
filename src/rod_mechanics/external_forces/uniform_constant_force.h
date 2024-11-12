#ifndef UNIFORM_CONSTANT_FORCE_H
#define UNIFORM_CONSTANT_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class UniformConstantForce : public BaseForce
{
  public:
    explicit UniformConstantForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~UniformConstantForce() override;

    void addForceToLimb(int limb_idx, const Vec3& force);

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    std::vector<std::pair<int, Vec3>> limb_force_pairs;
};

#endif  // UNIFORM_CONSTANT_FORCE_H
