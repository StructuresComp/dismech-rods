#include "implicitMidpoint.h"

implicitMidpoint::implicitMidpoint(const vector<shared_ptr<elasticRod>>& m_limbs,
                                   const vector<shared_ptr<elasticJoint>>& m_joints,
                                   const vector<shared_ptr<rodController>>& m_controllers,
                                   const shared_ptr<innerForces>& m_inner_forces,
                                   const shared_ptr<externalForces>& m_external_forces,
                                   double m_dt, double m_force_tol, double m_stol,
                                   int m_max_iter, int m_line_search,
                                   int m_adaptive_time_stepping, solverType m_solver_type) :
                                   backwardEuler(m_limbs, m_joints, m_controllers, m_inner_forces,
                                                 m_external_forces, m_dt, m_force_tol, m_stol, m_max_iter,
                                                 m_line_search, m_adaptive_time_stepping, m_solver_type)

{
}

implicitMidpoint::~implicitMidpoint() = default;


double implicitMidpoint::stepForwardInTime() {
    dt = orig_dt;
    for (const auto& limb : limbs) limb->updateGuess(0.01, 0.5 * dt);
    // Compute position at T=t+0.5dt
    dt = 2 * newtonMethod(0.5 * dt);
    for (const auto& limb : limbs) {
        // Compute velocity at T=t+0.5dt
        limb->u = (limb->x - limb->x0) / (0.5 * dt);

        // Compute position at T=t+dt
        limb->x = 2 * limb->x - limb->x0;
        limb->x0 = limb->x;

        // Compute velocity at T=t+dt
        limb->u = 2 * limb->u - limb->u0;
        limb->u0 = limb->u;
    }
    updateSystemForNextTimeStep();
    return dt;
}