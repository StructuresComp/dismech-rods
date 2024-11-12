#ifndef BASE_SOLVER_H
#define BASE_SOLVER_H

#include "global_definitions.h"
#include "mkl_pardiso.h"
#include "mkl_spblas.h"
#include "mkl_types.h"
#include "solver_types.h"

// Define the format to printf MKL_INT values
#if !defined(MKL_ILP64)
#define IFORMAT "%i"
#else
#define IFORMAT "%lli"
#endif

class ImplicitTimeStepper;

class BaseSolver
{
  public:
    BaseSolver(std::shared_ptr<ImplicitTimeStepper> stepper, SolverType solver_type);
    virtual ~BaseSolver();
    virtual void integrator() = 0;
    SolverType getSolverType();

  protected:
    MKL_INT nrhs;
    std::shared_ptr<ImplicitTimeStepper> stepper;

  private:
    SolverType solver_type;
};

#endif  // BASE_SOLVER_H
