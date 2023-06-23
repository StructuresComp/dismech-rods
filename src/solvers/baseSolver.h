#ifndef BASESOLVER_H
#define BASESOLVER_H

#include "../eigenIncludes.h"


class implicitTimeStepper;


enum solverType {
    PARDISO_SOLVER = 1,
    DGBSV_SOLVER
};


class baseSolver
{
public:
    baseSolver(shared_ptr<implicitTimeStepper> stepper, solverType solver_type);
    virtual ~baseSolver();

    virtual void integrator() = 0;

    shared_ptr<implicitTimeStepper> stepper;

    solverType solver_type;
};

#endif
