#ifndef BASETIMESTEPPER_H
#define BASETIMESTEPPER_H

#include "eigenIncludes.h"
#include "robotDescription.h"
#include "rod_mechanics/softRobots.h"
#include "rod_mechanics/forceContainer.h"
#include "controllers/rodController.h"


class baseTimeStepper : public enable_shared_from_this<baseTimeStepper>
{
public:
    baseTimeStepper(const shared_ptr<softRobots>& m_soft_robots,
                    const shared_ptr<forceContainer>& m_forces,
                    const simParams& sim_params);
    virtual ~baseTimeStepper();

    void addForce(int ind, double p, int limb_idx);

    virtual void initStepper();
    virtual void prepSystemForIteration();
    virtual void setZero();
    virtual void update();
    virtual void integrator() = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx) = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) = 0;
    virtual void updateSystemForNextTimeStep() = 0;
    virtual double stepForwardInTime() = 0;

    double* dx;
    double* force;
    Map<VectorXd> Force;
    Map<VectorXd> DX;
    MatrixXd Jacobian;

    int freeDOF;
    vector<int> offsets;
    int iter = 0;


protected:
    int mappedInd, mappedInd1, mappedInd2;
    int offset;
    double dt;
    double alpha = 1.0;

    vector<shared_ptr<elasticRod>>& limbs;
    vector<shared_ptr<elasticJoint>>& joints;
    vector<shared_ptr<rodController>>& controllers;
    shared_ptr<forceContainer> forces;
};

#endif
