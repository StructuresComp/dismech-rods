#ifndef BACKWARDEULER_H
#define BACKWARDEULER_H

#include "implicitTimeStepper.h"

class backwardEuler : public implicitTimeStepper
{
  public:
    backwardEuler(const shared_ptr<softRobots>& soft_robots,
                  const shared_ptr<forceContainer>& forces, const simParams& sim_params,
                  solverType solver_type);
    ~backwardEuler() override;

    void updateSystemForNextTimeStep() override;

    double newtonMethod(double dt) override;
    double lineSearch(double dt) override;

    double stepForwardInTime() override;

  private:
    double goldSteinLineSearch(double dt);
    double wolfeLineSearch(double dt);
};

#endif
