#ifndef BACKWARDEULER_H
#define BACKWARDEULER_H

#include "implicitTimeStepper.h"

class backwardEuler : public implicitTimeStepper
{
public:
    backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                  const vector<shared_ptr<elasticJoint>>& m_joints,
                  const vector<shared_ptr<rodController>>& m_controllers,
                  const shared_ptr<innerForces>& m_inner_forces,
                  const shared_ptr<externalForces>& m_external_forces,
                  double m_dt, double m_force_tol, double m_stol,
                  int m_max_iter, int m_line_search,
                  int m_adaptive_time_stepping, solverType m_solver_type);
    ~backwardEuler() override;

    void updateSystemForNextTimeStep() override;

    double newtonMethod(double dt) override;
    void lineSearch(double dt) override;
    double stepForwardInTime() override;

};


#endif
