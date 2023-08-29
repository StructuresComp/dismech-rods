#ifndef BASETIMESTEPPER_H
#define BASETIMESTEPPER_H

#include "controllers/rodController.h"
#include "rod_mechanics/elasticRod.h"
#include "eigenIncludes.h"

#include "rod_mechanics/inner_forces/innerForces.h"
#include "rod_mechanics/external_forces/externalForces.h"


class baseTimeStepper : public enable_shared_from_this<baseTimeStepper>
{
public:
    baseTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs,
                    const vector<shared_ptr<elasticJoint>>& m_joints,
                    const vector<shared_ptr<rodController>>& m_controllers,
                    shared_ptr<innerForces> m_inner_forces,
                    shared_ptr<externalForces> m_external_forces,
                    double m_dt);
    virtual ~baseTimeStepper();

    void addForce(int ind, double p, int limb_idx);

    void setupForceStepperAccess();
    virtual void prepSystemForIteration();
    virtual void setZero();
    virtual void update();
    virtual void initSolver() = 0;
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

    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    vector<shared_ptr<rodController>> controllers;

    shared_ptr<innerForces> inner_forces;
    shared_ptr<externalForces> external_forces;
};

#endif
