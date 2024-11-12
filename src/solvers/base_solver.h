#ifndef BASE_SOLVER_H
#define BASE_SOLVER_H

#include "global_definitions.h"
#include "solver_types.h"

class ImplicitTimeStepper;

class BaseSolver
{
  public:
    BaseSolver(const std::shared_ptr<ImplicitTimeStepper>& stepper, SolverType solver_type);
    virtual ~BaseSolver();
    virtual void integrator() = 0;
    SolverType getSolverType();

  protected:
    int nrhs;
    std::shared_ptr<ImplicitTimeStepper> stepper;

  private:
    SolverType solver_type;
};

#endif  // BASE_SOLVER_H
