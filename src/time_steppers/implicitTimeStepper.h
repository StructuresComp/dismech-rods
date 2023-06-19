#ifndef IMPLICITTIMESTEPPER_H
#define IMPLICITTIMESTEPPER_H


#include "baseTimeStepper.h"
#include "mkl_pardiso.h"
#include "mkl_types.h"
#include "mkl_spblas.h"

// Define the format to printf MKL_INT values
#if !defined(MKL_ILP64)
#define IFORMAT "%i"
#else
#define IFORMAT "%lli"
#endif

class implicitTimeStepper : public baseTimeStepper
{
public:
    implicitTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs,
                        const vector<shared_ptr<elasticJoint>>& m_joints,
                        const vector<shared_ptr<rodController>>& m_controllers,
                        shared_ptr<elasticStretchingForce> m_stretchForce,
                        shared_ptr<elasticBendingForce> m_bendingForce,
                        shared_ptr<elasticTwistingForce> m_twistingForce,
                        shared_ptr<inertialForce> m_inertialForce,
                        shared_ptr<externalGravityForce> m_gravityForce,
                        shared_ptr<dampingForce> m_dampingForce,
                        shared_ptr<floorContactForce> m_floorContactForce,
                        double m_dt, double m_force_tol, double m_stol,
                        int m_max_iter, int m_line_search);
    ~implicitTimeStepper() override;

    double* getJacobian() override;
    void addJacobian(int ind1, int ind2, double p, int limb_idx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) override;
    void setZero() override;
    void update() override;

    void prepSystemForIteration() override;
    virtual void newtonMethod(double dt) = 0;
    virtual void lineSearch(double dt) = 0;
    void pardisoSolver();
protected:
    double force_tol;
    double stol;
    int max_iter;
    int line_search;
private:
    double *jacobian;

    // utility variables for dgbsv solver
    int kl, ku;
    int jacobianLen;
    int nrhs;
    int *ipiv;
    int info;
    int ldb;
    int NUMROWS;

    // utility variables for PARDISO solver
    MKL_INT* ia;
    vector<pair<int, int>> non_zero_elements;

    /* Internal solver memory pointer pt, */
    /* 32-bit: int pt[64]; 64-bit: long int pt[64] */
    /* or void *pt[64] should be OK on both architectures */
    void *pt[64];

    /* Pardiso control parameters. */
    MKL_INT mtype;
    MKL_INT iparm[64];
    MKL_INT maxfct, mnum, phase, error, msglvl;
};


#endif
