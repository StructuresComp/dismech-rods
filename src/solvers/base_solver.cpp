#include "base_solver.h"

BaseSolver::BaseSolver(const std::weak_ptr<ImplicitTimeStepper>& stepper, SolverType solver_type)
    : weak_implicit_stepper(stepper), solver_type(solver_type) {
    nrhs = 1; /* Number of right hand sides. */
}

BaseSolver::~BaseSolver() = default;

SolverType BaseSolver::getSolverType() {
    return solver_type;
}
