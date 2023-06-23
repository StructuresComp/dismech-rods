#include "baseSolver.h"


baseSolver::baseSolver(shared_ptr<implicitTimeStepper> m_stepper,
                       solverType m_solver_type) : stepper(m_stepper),
                       solver_type(m_solver_type) {

}



baseSolver::~baseSolver() = default;
