#ifndef TIMESTEPPER_H
#define TIMESTEPPER_H

#include "elasticRod.h"
#include "../eigenIncludes.h"
#include "mkl_pardiso.h"
#include "mkl_types.h"
#include "mkl_spblas.h"

// Define the format to printf MKL_INT values
#if !defined(MKL_ILP64)
#define IFORMAT "%i"
#else
#define IFORMAT "%lli"
#endif


class timeStepper
{
public:
    timeStepper(elasticRod &m_rod);
    ~timeStepper();
    double* getForce();
    double* getJacobian();
    void setZero();
    void addForce(int ind, double p);
    void addJacobian(int ind1, int ind2, double p);
    void integrator();

    void pardisoSolver();

    VectorXd force;

    void update();
    VectorXd Force;
    MatrixXd Jacobian;
    VectorXd DX;
    double *dx;

private:
    elasticRod *rod;
    int kl, ku, freeDOF;

    double *totalForce;
    double *jacobian;

    // utility variables
    int mappedInd, mappedInd1, mappedInd2;
    int row, col, offset;
    int NUMROWS;
    int jacobianLen;
    int nrhs;
    int *ipiv;
    int info;
    int ldb;
};

#endif
