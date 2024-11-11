#ifndef PARDISO_SOLVER_H
#define PARDISO_SOLVER_H

#include "base_solver.h"

class PardisoSolver : public BaseSolver
{
  public:
    explicit PardisoSolver(shared_ptr<ImplicitTimeStepper> stepper);
    ~PardisoSolver() override;

    void integrator() override;

  private:
    /* Internal solver memory pointer pt, */
    /* 32-bit: int pt[64]; 64-bit: long int pt[64] */
    /* or void *pt[64] should be OK on both architectures */
    void* pt[64];

    /* Pardiso control parameters. */
    MKL_INT mtype;
    MKL_INT iparm[64];
    MKL_INT maxfct, mnum, phase, error, msglvl;
};

#endif  // PARDISO_SOLVER_H
