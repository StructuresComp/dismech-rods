#include "backwardEuler.h"

backwardEuler::backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                             const vector<shared_ptr<elasticJoint>>& m_joints,
                             const vector<shared_ptr<rodController>>& m_controllers,
                             shared_ptr<elasticStretchingForce> m_stretch_force,
                             shared_ptr<elasticBendingForce> m_bending_force,
                             shared_ptr<elasticTwistingForce> m_twisting_force,
                             shared_ptr<inertialForce> m_inertial_force,
                             shared_ptr<externalGravityForce> m_gravity_force,
                             shared_ptr<dampingForce> m_damping_force,
                             shared_ptr<floorContactForce> m_floor_contact_force,
                             double m_dt, double m_force_tol, double m_stol,
                             int m_max_iter, int m_line_search) :
                             implicitTimeStepper(m_limbs, m_joints, m_controllers, m_stretch_force, m_bending_force,
                                                 m_twisting_force, m_inertial_force, m_gravity_force,
                                                 m_damping_force, m_floor_contact_force, m_dt,
                                                 m_force_tol, m_stol, m_max_iter, m_line_search)

{
}

backwardEuler::~backwardEuler() = default;


void backwardEuler::integrator()
{
    pardisoSolver();
}


void backwardEuler::newtonMethod(double dt) {
    double normf;
    double normf0 = 0;
    bool solved = false;
    double ratio = 1.1;
    iter = 0;

    floor_contact_force->reset_slip_tol();

    while (!solved) {
        prepSystemForIteration();

        inertial_force->computeFi(dt);
        inertial_force->computeJi(dt);

        stretching_force->computeFs();
        stretching_force->computeJs();

        bending_force->computeFb();
        bending_force->computeJb();

        twisting_force->computeFt();
        twisting_force->computeJt();

        gravity_force->computeFg();
        gravity_force->computeJg();

        damping_force->computeFd(dt);
        damping_force->computeJd(dt);

        floor_contact_force->computeFfJf(dt);

//        m_collisionDetector->detectCollisions();
//        if (iter == 0) {
//            m_contactPotentialIMC->updateContactStiffness();
//        }

//        m_contactPotentialIMC->computeFcJc();

        // Compute norm of the force equations.
        normf = 0;
        for (int i = 0; i < freeDOF; i++) {
            normf += totalForce[i] * totalForce[i];
        }
        normf = sqrt(normf);

        if (iter == 0) {
            normf0 = normf;
        }

        if (normf <= force_tol || (iter > 0 && normf <= normf0 * stol)) {
            solved = true;
            iter++;
        }

        // If sim can't converge, relax friction rigidity and redo timestep.
        // This is usually the issue
        if (iter != 0 && iter % 50 == 0) {
            for (const auto& limb : limbs) {
                limb->x = limb->x0;
                limb->updateGuess(0.01, dt);
            }
            floor_contact_force->change_slip_tol(ratio);
            ratio += 0.1;
            iter++;
            continue;
        }

        if (!solved) {
            integrator(); // Solve equations of motion
            if (line_search) lineSearch(dt);

            int limb_idx = 0;
            for (const auto& limb : limbs) {
                limb->updateNewtonX(dx, offsets[limb_idx], alpha);
                limb_idx++;
            }
            iter++;
        }

        // Exit if unable to converge
        if (iter > max_iter) {
            cout << "No convergence after " << max_iter << " iterations" << endl;
            exit(1);
        }

    }
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
        inertial_force->computeFi(dt);
        stretching_force->computeFs();
        bending_force->computeFb();
        twisting_force->computeFt();
        gravity_force->computeFg();
        damping_force->computeFd(dt);
        floor_contact_force->computeFf(dt);
//        m_collisionDetector->detectCollisions();
//        m_contactPotentialIMC->computeFc();

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


void backwardEuler::stepForwardInTime() {
    for (const auto& limb : limbs) limb->updateGuess(0.01, dt);
    newtonMethod(dt);
    updateSystemForNextTimeStep();
}


void backwardEuler::updateSystemForNextTimeStep() {
    prepSystemForIteration();

    for (const auto& controller : controllers) {
        controller->updateTimestep(dt);
    }

    for (const auto& limb : limbs) {
        // Update velocity
        limb->u = (limb->x - limb->x0) / dt;

        // Update x0
        limb->x0 = limb->x;

        // Update reference directors
        limb->d1_old = limb->d1;
        limb->d2_old = limb->d2;

        // Update necessary info for time parallel
        limb->tangent_old = limb->tangent;
        limb->refTwist_old = limb->refTwist;
    }

    // Do the same updates as above for joints
    for (const auto& joint : joints) {
        joint->u = (joint->x - joint->x0) / dt;
        joint->x0 = joint->x;
        joint->d1_old = joint->d1;
        joint->d2_old = joint->d2;
        joint->tangents_old = joint->tangents;
        joint->ref_twist_old = joint->ref_twist;
    }
}

