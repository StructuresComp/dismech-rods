#ifndef BACKWARD_EULER_H
#define BACKWARD_EULER_H

#include "implicit_time_stepper.h"

class BackwardEuler : public ImplicitTimeStepper
{
  public:
    BackwardEuler(const shared_ptr<SoftRobots>& soft_robots,
                  const shared_ptr<ForceContainer>& forces, const SimParams& sim_params,
                  SolverType solver_type);
    ~BackwardEuler() override;

    void updateSystemForNextTimeStep() override;
    double stepForwardInTime() override;

  protected:
    double newtonMethod(double dt) override;
    double lineSearch(double dt) override;

  private:
    double goldSteinLineSearch(double dt);
    double wolfeLineSearch(double dt);
};

#endif  // BACKWARD_EULER_H
