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
                        shared_ptr<elasticStretchingForce> m_stretchForce,
                        shared_ptr<elasticBendingForce> m_bendingForce,
                        shared_ptr<elasticTwistingForce> m_twistingForce,
                        shared_ptr<inertialForce> m_inertialForce,
                        shared_ptr<externalGravityForce> m_gravityForce,
                        shared_ptr<dampingForce> m_dampingForce,
                        shared_ptr<floorContactForce> m_floorContactForce);
    virtual ~implicitTimeStepper();

    double* getJacobian() override;
    void addJacobian(int ind1, int ind2, double p, int limb_idx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) override;
    void setZero() override;
    void update() override;

    void pardisoSolver();
private:
    double *jacobian;

    // utility variables for PARDISO solver
    int kl, ku;
    int jacobianLen;
    int nrhs;
    int *ipiv;
    int info;
    int ldb;
    int NUMROWS;
};


#endif
