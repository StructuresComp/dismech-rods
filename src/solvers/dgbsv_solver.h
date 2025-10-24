#ifndef DGBSV_SOLVER_H
#define DGBSV_SOLVER_H

#include "base_solver.h"

class DGBSVSolver : public BaseSolver
{
  public:
    explicit DGBSVSolver(const std::weak_ptr<ImplicitTimeStepper>& stepper);
    ~DGBSVSolver() override;

    void integrator() override;

    int kl;  // lower diagonals
    int ku;  // upper diagonals
    int NUMROWS;

  private:
    int info;
};

#endif  // DGBSV_SOLVER_H
