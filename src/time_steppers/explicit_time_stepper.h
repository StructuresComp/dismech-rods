#ifndef EXPLICIT_TIME_STEPPER_H
#define EXPLICIT_TIME_STEPPER_H

#include "base_time_stepper.h"

class ExplicitTimeStepper : public BaseTimeStepper
{
  public:
    ExplicitTimeStepper(const std::shared_ptr<SoftRobots>& soft_robots,
                        const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params);
    ~ExplicitTimeStepper() override;

    void prepSystemForIteration() override;
    void integrator() override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) override;

  protected:
    std::vector<VecX> inverse_masses;

  private:
    void constructInverseMassVector();
};

#endif
