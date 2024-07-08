#include "backwardEuler.h"

backwardEuler::backwardEuler(const shared_ptr<softRobots>& soft_robots,
                             const shared_ptr<forceContainer>& forces,
                             const simParams& sim_params, solverType solver_type) :
                             implicitTimeStepper(soft_robots, forces, sim_params, solver_type)
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

        forces->computeForcesAndJacobian(dt);

        // Compute norm of the force equations.
        normf = Force.norm();

        if (iter == 0) {
            normf0 = normf;
        }

        // Force tolerance check
        if (normf <= normf0 * ftol) {
            solved = true;
            iter++;
            continue;
        }

        // If sim can't converge, apply adaptive time stepping if enabled.
        if (adaptive_time_stepping && iter != 0 && iter % adaptive_time_stepping_threshold == 0) {
            dt *= 0.5;
            for (const auto& limb : limbs) {
                limb->updateGuess(0.01, dt);
            }
            iter++;
            continue;
        }

        // Solve equations of motion
        integrator();
        if (line_search) lineSearch_gs(dt);

        // Apply Newton update
        double curr_dx;
        double max_dx = 0;
        int limb_idx = 0;
        for (const auto& limb : limbs) {
            // TODO: make sure that joint dx's are properly included in this?
            curr_dx = limb->updateNewtonX(dx, offsets[limb_idx], alpha);

            // Record max change dx
            if (curr_dx > max_dx) {
                max_dx = curr_dx;
            }

            limb_idx++;
        }

        // Dynamics tolerance check
        if (max_dx / dt < dtol) {
            solved = true;
            iter++;
            continue;
        }

        iter++;

        // Exit if unable to converge
        if (iter > max_iter) {
            if (terminate_at_max) {
                cout << "No convergence after " << max_iter << " iterations" << endl;
                exit(1);
            }
            else {
                solved = true;
                continue;
            }
        }
    }
    return dt;
}


void backwardEuler::lineSearch_gs(double dt) {
    // store current x
    for (auto& limb : limbs) {
        limb->x_ls = limb->x;
    }
    for (auto& joint : joints) {
        joint->x_ls = joint->x;
    }
    // Initialize an interval for optimal learning rate alpha
    double amax = 2;
    double amin = 1e-3;
//    double amin = 1e-5;
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
    alpha = a;
    while (!success) {
        int limb_idx = 0;
        for (auto& joint : joints) {
            joint->x = joint->x_ls;
        }
        for (auto& limb : limbs) {
            limb->x= limb->x_ls;
            limb->updateNewtonX(dx, offsets[limb_idx], a);
            limb_idx++;
        }
        prepSystemForIteration();

        // Compute the forces
        forces->computeForces(dt);

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
            else {
                a = 10 * a;
            }
        }
        if (a > amax || a < amin) {
            break;
        }
//        if (iter_l > 100) {
//            break;
//        }
//        iter_l++;
    }
    for (auto& limb : limbs) {
        limb->x = limb->x_ls;
    }
    for (auto& joint : joints) {
        joint->x = joint->x_ls;
    }
    alpha = a;
}


void backwardEuler::lineSearch_wolfe(double dt) {
    // store current x
    for (auto& limb : limbs) {
        limb->x_ls = limb->x;
    }
    for (auto& joint : joints) {
        joint->x_ls = joint->x;
    }
    double c1 = 1e-4;
    double c2 = 0.9;

    //compute the slope initially
    double q0 = 0.5 * pow(Force.norm(), 2);
    double dq0 = -(Force.transpose() * Jacobian * DX)(0);
    double a = 1;
    alpha = a;
    for (int i = 0; i < 10; i++)
    {
        int limb_idx = 0;
        for (auto& joint : joints) {
            joint->x = joint->x_ls;
        }
        for (auto& limb : limbs) {
            limb->x= limb->x_ls;
            limb->updateNewtonX(dx, offsets[limb_idx], a);
            limb_idx++;
        }
        prepSystemForIteration();
        // Compute the forces
        forces->computeForcesAndJacobian(dt);
        
        double q = 0.5 * pow(Force.norm(), 2);

        if (q <= q0 + c1 * alpha * dq0){
            double dq = -(Force.transpose() * Jacobian * DX)(0);
            if (dq >= c2 * dq0)
            {
                break;
            }
        }
        a /= 2.0;

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

    // Newton Guess. Just use approximately last solution
    for (const auto& limb : limbs) limb->updateGuess(0.01, dt);

    // Perform collision detection if contact is enabled
    if (forces->cf) forces->cf->broadPhaseCollisionDetection();

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
        controller->updateTimeStep(dt);
    }

    for (const auto& limb : limbs) {
        // Update reference directors
        limb->d1_old = limb->d1;
        limb->d2_old = limb->d2;

        // Update necessary info for time parallel
        limb->tangent_old = limb->tangent;
        limb->ref_twist_old = limb->ref_twist;
    }

    // Do the same updates as above for joints
    for (const auto& joint : joints) {
        joint->d1_old = joint->d1;
        joint->d2_old = joint->d2;
        joint->tangents_old = joint->tangents;
        joint->ref_twist_old = joint->ref_twist;
    }
}
