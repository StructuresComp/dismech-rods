#ifndef BASESOLVER_H
#define BASESOLVER_H

#include "globalDefinitions.h"
#include "solverTypes.h"
#include "mkl_pardiso.h"
#include "mkl_types.h"
#include "mkl_spblas.h"

// Define the format to printf MKL_INT values
#if !defined(MKL_ILP64)
#define IFORMAT "%i"
#else
#define IFORMAT "%lli"
#endif


class implicitTimeStepper;


class baseSolver
{
public:
    baseSolver(shared_ptr<implicitTimeStepper> stepper, solverType solver_type);
    virtual ~baseSolver();
    virtual void integrator() = 0;
    solverType getSolverType();
protected:
    MKL_INT nrhs;
    shared_ptr<implicitTimeStepper> stepper;
private:
    solverType solver_type;
};

#endif
