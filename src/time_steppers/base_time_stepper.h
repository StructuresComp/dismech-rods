#ifndef BASE_TIME_STEPPER_H
#define BASE_TIME_STEPPER_H

#include "controllers/base_controller.h"
#include "global_definitions.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

class BaseTimeStepper : public enable_shared_from_this<BaseTimeStepper>
{
  public:
    BaseTimeStepper(const shared_ptr<SoftRobots>& m_soft_robots,
                    const shared_ptr<ForceContainer>& m_forces, const SimParams& sim_params);
    virtual ~BaseTimeStepper();

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

    vector<shared_ptr<ElasticRod>>& limbs;
    vector<shared_ptr<ElasticJoint>>& joints;
    vector<shared_ptr<BaseController>>& controllers;
    shared_ptr<ForceContainer> forces;
};

#endif  // BASE_TIME_STEPPER_H
