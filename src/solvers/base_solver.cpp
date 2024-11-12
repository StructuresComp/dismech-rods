#include "base_solver.h"

BaseSolver::BaseSolver(std::shared_ptr<ImplicitTimeStepper> m_stepper, SolverType m_solver_type)
    : stepper(m_stepper), solver_type(m_solver_type) {
    nrhs = 1; /* Number of right hand sides. */
}

BaseSolver::~BaseSolver() = default;

SolverType BaseSolver::getSolverType() {
    return solver_type;
}
