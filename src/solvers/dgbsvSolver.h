#ifndef DGBSV_SOLVER_H
#define DGBSV_SOLVER_H

#include "baseSolver.h"

class dgbsvSolver : public baseSolver
{
  public:
    explicit dgbsvSolver(shared_ptr<implicitTimeStepper> stepper);
    ~dgbsvSolver() override;

    void integrator() override;

    MKL_INT kl;  // lower diagonals
    MKL_INT ku;  // upper diagonals
    MKL_INT NUMROWS;

  private:
    MKL_INT info;
};

#endif
