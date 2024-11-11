#ifndef DGBSV_SOLVER_H
#define DGBSV_SOLVER_H

#include "base_solver.h"

class DGBSVSolver : public BaseSolver
{
  public:
    explicit DGBSVSolver(shared_ptr<ImplicitTimeStepper> stepper);
    ~DGBSVSolver() override;

    void integrator() override;

    MKL_INT kl;  // lower diagonals
    MKL_INT ku;  // upper diagonals
    MKL_INT NUMROWS;

  private:
    MKL_INT info;
};

#endif  // DGBSV_SOLVER_H
