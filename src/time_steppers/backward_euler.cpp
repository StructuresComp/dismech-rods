#include "backward_euler.h"
#include "controllers/base_controller.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/external_forces/contact_force.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

BackwardEuler::BackwardEuler(const std::shared_ptr<SoftRobots>& soft_robots,
                             const std::shared_ptr<ForceContainer>& forces,
                             const SimParams& sim_params, SolverType solver_type)
    : ImplicitTimeStepper(soft_robots, forces, sim_params, solver_type) {
}

BackwardEuler::~BackwardEuler() = default;

double BackwardEuler::newtonMethod(double dt) {
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
        alpha = lineSearch(dt);

        // Apply Newton update
        double curr_dx;
        double max_dx = 0;
        int limb_idx = 0;
        for (const auto& limb : limbs) {
            // TODO: make sure that joint dx's are properly included in this?
            curr_dx = limb->updateNewtonX(dx.data(), offsets[limb_idx], alpha);

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

        // Exit if unable to converge
        if (iter >= max_iter - 1) {
            if (terminate_at_max) {
                std::cout << "No convergence after " << max_iter << " iterations" << std::endl;
                exit(1);
            }
            else {
                solved = true;
                continue;
            }
        }
        iter++;
    }
    return dt;
}

double BackwardEuler::lineSearch(double dt) {
    switch (line_search_type) {
        case NO_LS:
            return 1.0;
        case GOLDSTEIN:
            return goldSteinLineSearch(dt);
        case WOLFE:
            return wolfeLineSearch(dt);
        default:
            throw std::invalid_argument("Invalid line search type");
    }
}

double BackwardEuler::goldSteinLineSearch(double dt) {
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
    double al = 0;
    double au = 1;
    double m2 = 0.9;
    double m1 = 0.1;

    // Initial step size
    double a = 1;

    // compute the slope initially
    double q0 = 0.5 * pow(Force.norm(), 2);
    double dq0 = -(Force.transpose() * Jacobian * DX)(0);

    int curr_iters = 0;
    int max_iters = 30;
    bool success = false;
    while (!success) {
        if (curr_iters >= max_iters) {
            break;
        }
        curr_iters++;
        int limb_idx = 0;
        for (auto& joint : joints) {
            joint->x = joint->x_ls;
        }

        for (auto& limb : limbs) {
            limb->x = limb->x_ls;
            limb->updateNewtonX(dx.data(), offsets[limb_idx], a);
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
    }

    for (auto& limb : limbs) {
        limb->x = limb->x_ls;
    }
    for (auto& joint : joints) {
        joint->x = joint->x_ls;
    }
    return a;
}

double BackwardEuler::wolfeLineSearch(double dt) {
    // store current x
    for (auto& limb : limbs) {
        limb->x_ls = limb->x;
    }
    for (auto& joint : joints) {
        joint->x_ls = joint->x;
    }

    // Initialize constants
    double c1 = 1e-4;  // Armijo condition
    double c2 = 0.9;   // Curvature condition

    // Initial step size
    double a = 1;

    // compute the slope initially
    double q0 = 0.5 * pow(Force.norm(), 2);
    double dq0 = -(Force.transpose() * Jacobian * DX)(0);

    for (int i = 0; i < 10; i++) {
        int limb_idx = 0;
        for (auto& joint : joints) {
            joint->x = joint->x_ls;
        }
        for (auto& limb : limbs) {
            limb->x = limb->x_ls;
            limb->updateNewtonX(dx.data(), offsets[limb_idx], a);
            limb_idx++;
        }
        prepSystemForIteration();
        // Compute the forces
        forces->computeForcesAndJacobian(dt);
        double q = 0.5 * pow(Force.norm(), 2);

        // Check Armijo condition
        if (q <= q0 + c1 * a * dq0) {
            double dq = -(Force.transpose() * Jacobian * DX)(0);
            // Check curvature condition
            if (dq >= c2 * dq0) {
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
    return a;
}

double BackwardEuler::stepForwardInTime() {
    dt = orig_dt;

    // Newton Guess. Just use approximately last solution
    for (const auto& limb : limbs)
        limb->updateGuess(0.01, dt);

    // Perform collision detection if contact is enabled
    if (forces->cf)
        forces->cf->broadPhaseCollisionDetection();

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

void BackwardEuler::updateSystemForNextTimeStep() {
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
