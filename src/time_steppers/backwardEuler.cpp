#include "backwardEuler.h"

backwardEuler::backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                             const vector<shared_ptr<elasticJoint>>& m_joints,
                             const vector<shared_ptr<rodController>>& m_controllers,
                             const shared_ptr<innerForces>& m_inner_forces,
                             const shared_ptr<externalForces>& m_external_forces,
                             double m_dt, double m_force_tol, double m_stol,
                             int m_max_iter, int m_line_search, int m_adaptive_time_stepping, solverType m_solver_type) :
                             implicitTimeStepper(m_limbs, m_joints, m_controllers, m_inner_forces, m_external_forces,
                                                 m_dt, m_force_tol, m_stol, m_max_iter, m_line_search,
                                                 m_adaptive_time_stepping, m_solver_type)

{
}

backwardEuler::~backwardEuler() = default;


double backwardEuler::newtonMethod(double dt) {
    double normf;
    double normf0 = 0;
    bool solved = false;
    iter = 0;

    while (!solved) {
        prepSystemForIteration();

        inner_forces->computeForcesAndJacobian(dt);
        external_forces->computeForcesAndJacobian(dt);

        // Compute norm of the force equations.
        // TODO: replace with eigen operation
        normf = 0;
        for (int i = 0; i < freeDOF; i++) {
            normf += force[i] * force[i];
        }
        normf = sqrt(normf);

        if (iter == 0) {
            normf0 = normf;
        }

        if (normf <= force_tol || (iter > 0 && normf <= normf0 * stol)) {
            solved = true;
            iter++;
            continue;
        }

        // If sim can't converge, apply adaptive time stepping if enabled.
        if (adaptive_time_stepping && iter != 0 && iter % adaptive_time_stepping_threshold == 0) {
            dt *= 0.5;
            for (const auto& limb : limbs) {
                limb->x = limb->x0;
                limb->updateGuess(0.01, dt);
            }
            iter++;
            continue;
        }

        // Solve equations of motion
        integrator();
        if (line_search) lineSearch(dt);

        // Apply Newton update
        int limb_idx = 0;
        for (const auto& limb : limbs) {
            limb->updateNewtonX(dx, offsets[limb_idx], alpha);
            limb_idx++;
        }
        iter++;

        // Exit if unable to converge
        if (iter > max_iter) {
            cout << "No convergence after " << max_iter << " iterations" << endl;
            exit(1);
        }

    }
    return dt;
}


void backwardEuler::lineSearch(double dt) {
    // store current x
    for (auto& limb : limbs) {
        limb->x_ls = limb->x;
    }
    for (auto& joint : joints) {
        joint->x_ls = joint->x;
    }
    // Initialize an interval for optimal learning rate alpha
    double amax = 2;
//    double amin = 1e-3;
    double amin = 1e-5;
    double al = 0;
    double au = 1;

    double a = 1;

    //compute the slope initially
    double q0 = 0.5 * pow(Force.norm(), 2);
    double dq0 = -(Force.transpose() * Jacobian * DX)(0);

    bool success = false;
    double m2 = 0.9;
    double m1 = 0.1;
    int iter_l = 0;
    while (!success) {
        int limb_idx = 0;
        for (auto& joint : joints) {
            joint->x = joint->x_ls;
        }
        for (auto& limb : limbs) {
            limb->x= limb->x_ls;
            limb->updateNewtonX(dx, offsets[limb_idx], alpha);
            limb_idx++;
        }

        prepSystemForIteration();

        // Compute the forces
        inner_forces->computeForces(dt);
        external_forces->computeForces(dt);

        double q = 0.5 * pow(Force.norm(), 2);

        double slope = (q - q0) / a;

        if (slope >= m2 * dq0 && slope <= m1 * dq0) {
            success = true;
        }
        else {
            if (slope < m2 * dq0) {
                al = a;
            }
            else {
                au = a;
            }

            if (au < amax) {
                a = 0.5 * (al + au);
            }
//            else {
//                a = 10 * a;
//            }
        }
        if (a > amax || a < amin) {
            break;
        }
        if (iter_l > 100) {
            break;
        }
        iter_l++;
    }
    for (auto& limb : limbs) {
        limb->x = limb->x_ls;
    }
    for (auto& joint : joints) {
        joint->x = joint->x_ls;
    }
    alpha = a;
}


double backwardEuler::stepForwardInTime() {
    dt = orig_dt;
    for (const auto& limb : limbs) limb->updateGuess(0.01, dt);
    dt = newtonMethod(dt);

    // Update limbs
    for (const auto& limb : limbs) {
        // Update velocity
        limb->u = (limb->x - limb->x0) / dt;
        // Update start position
        limb->x0 = limb->x;
    }

    updateSystemForNextTimeStep();
    return dt;
}


void backwardEuler::updateSystemForNextTimeStep() {
    prepSystemForIteration();

    for (const auto& controller : controllers) {
        controller->updateTimestep(dt);
    }

    for (const auto& limb : limbs) {
        // Update reference directors
        limb->d1_old = limb->d1;
        limb->d2_old = limb->d2;

        // Update necessary info for time parallel
        limb->tangent_old = limb->tangent;
        limb->refTwist_old = limb->refTwist;
    }

    // Do the same updates as above for joints
    for (const auto& joint : joints) {
        joint->d1_old = joint->d1;
        joint->d2_old = joint->d2;
        joint->tangents_old = joint->tangents;
        joint->ref_twist_old = joint->ref_twist;
    }
}

