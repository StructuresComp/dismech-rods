#ifndef BASETimeStepper_H
#define BASETimeStepper_H

#include "../rod_mechanics/elasticRod.h"
#include "../eigenIncludes.h"


class baseTimeStepper
{
public:
    baseTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs);
    ~baseTimeStepper();

    double* getForce();
    void addForce(int ind, double p, int limb_idx);

    virtual void setZero();
    virtual void update();
    virtual void integrator() = 0;
    virtual double* getJacobian() = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx) = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) = 0;

    VectorXd Force;
    MatrixXd Jacobian;
    VectorXd DX;
    double *dx;

    int freeDOF;
    vector<int> offsets;

protected:
    int mappedInd, mappedInd1, mappedInd2;
    int offset;
    vector<shared_ptr<elasticRod>> limbs;

private:
    double *totalForce;
};

#endif
