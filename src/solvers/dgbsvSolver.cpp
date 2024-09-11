#include "dgbsvSolver.h"
#include "../time_steppers/implicitTimeStepper.h"



dgbsvSolver::dgbsvSolver(shared_ptr<implicitTimeStepper> stepper) :
    baseSolver(stepper, solverType::DGBSV_SOLVER)
{
    kl = 10;  // lower diagonals
    ku = 10;  // upper diagonals

    NUMROWS = 2 * kl + ku + 1;

    info = 0;
}


dgbsvSolver::~dgbsvSolver() = default;


void dgbsvSolver::integrator() {
    MKL_INT n = stepper->freeDOF;
    MKL_INT ipiv[n];
    dgbsv_(&n, &kl, &ku, &nrhs, stepper->dgbsv_jacobian, &NUMROWS, ipiv, stepper->force, &n, &info);
    for (int i = 0; i < n; i++) {
        stepper->dx[i] = stepper->force[i];
    }
}
