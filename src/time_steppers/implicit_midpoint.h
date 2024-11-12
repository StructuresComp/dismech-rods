#ifndef IMPLICIT_MIDPOINT_H
#define IMPLICIT_MIDPOINT_H

#include "backward_euler.h"

class ImplicitMidpoint : public BackwardEuler
{
  public:
    ImplicitMidpoint(const std::shared_ptr<SoftRobots>& soft_robots,
                     const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params,
                     SolverType solver_type);
    ~ImplicitMidpoint() override;

    double stepForwardInTime() override;
};

#endif  // IMPLICIT_MIDPOINT_H
