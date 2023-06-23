#ifndef PARDISO_SOLVER_H
#define PARDISO_SOLVER_H

#include "baseSolver.h"
#include "mkl_pardiso.h"
#include "mkl_types.h"
#include "mkl_spblas.h"


// Define the format to printf MKL_INT values
#if !defined(MKL_ILP64)
#define IFORMAT "%i"
#else
#define IFORMAT "%lli"
#endif


class pardisoSolver : public baseSolver
{
public:
    pardisoSolver(shared_ptr<implicitTimeStepper> stepper);

    void integrator();

private:
    /* Internal solver memory pointer pt, */
    /* 32-bit: int pt[64]; 64-bit: long int pt[64] */
    /* or void *pt[64] should be OK on both architectures */
    void *pt[64];

    /* Pardiso control parameters. */
    MKL_INT mtype;
    MKL_INT iparm[64];
    MKL_INT maxfct, mnum, nrhs, phase, error, msglvl;
};


#endif
