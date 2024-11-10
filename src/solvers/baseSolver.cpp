#include "baseSolver.h"

baseSolver::baseSolver(shared_ptr<implicitTimeStepper> m_stepper, solverType m_solver_type)
    : stepper(m_stepper), solver_type(m_solver_type) {
    nrhs = 1; /* Number of right hand sides. */
}

baseSolver::~baseSolver() = default;

solverType baseSolver::getSolverType() {
    return solver_type;
}
