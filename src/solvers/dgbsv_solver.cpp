#include "dgbsv_solver.h"
#include "time_steppers/implicit_time_stepper.h"

DGBSVSolver::DGBSVSolver(const std::weak_ptr<ImplicitTimeStepper>& stepper)
    : BaseSolver(stepper, SolverType::DGBSV_SOLVER) {
    kl = 10;  // lower diagonals
    ku = 10;  // upper diagonals

    NUMROWS = 2 * kl + ku + 1;

    info = 0;
}

DGBSVSolver::~DGBSVSolver() = default;

void DGBSVSolver::integrator() {
    auto stepper = weak_implicit_stepper.lock();
    int n = stepper->freeDOF;
    int ipiv[n];
    dgbsv_(&n, &kl, &ku, &nrhs, stepper->dgbsv_jacobian.data(), &NUMROWS, ipiv,
           stepper->Force.data(), &n, &info);
    for (int i = 0; i < n; i++) {
        stepper->DX[i] = stepper->Force[i];
    }
}
