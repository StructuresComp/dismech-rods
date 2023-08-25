#ifndef IMPLICITMIDPOINT_H
#define IMPLICITMIDPOINT_H

#include "backwardEuler.h"

class implicitMidpoint : public backwardEuler
{
public:
    implicitMidpoint(const vector<shared_ptr<elasticRod>>& m_limbs,
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
                     int m_max_iter, int m_line_search,
                     int m_adaptive_time_stepping, solverType m_solver_type);
    ~implicitMidpoint() override;

    double stepForwardInTime() override;
};

#endif
