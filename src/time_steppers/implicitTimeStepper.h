#ifndef IMPLICITTIMESTEPPER_H
#define IMPLICITTIMESTEPPER_H

#include "baseTimeStepper.h"
#include "mkl_types.h"

class baseSolver;


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
    void integrator() override;
    void initSolver() override;

    void prepSystemForIteration() override;
    virtual void newtonMethod(double dt) = 0;
    virtual void lineSearch(double dt) = 0;


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

protected:
    double force_tol;
    double stol;
    int max_iter;
    int line_search;
private:
    shared_ptr<implicitTimeStepper> shared_from_this();

    double *jacobian;
    unique_ptr<baseSolver> solver;
};


#endif
