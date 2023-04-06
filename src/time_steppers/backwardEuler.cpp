#include "backwardEuler.h"

backwardEuler::backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                             const vector<shared_ptr<elasticJoint>>& m_joints,
                             shared_ptr<elasticStretchingForce> m_stretch_force,
                             shared_ptr<elasticBendingForce> m_bending_force,
                             shared_ptr<elasticTwistingForce> m_twisting_force,
                             shared_ptr<inertialForce> m_inertial_force,
                             shared_ptr<externalGravityForce> m_gravity_force,
                             shared_ptr<dampingForce> m_damping_force,
                             shared_ptr<floorContactForce> m_floor_contact_force,
                             double m_force_tol, double m_stol, int m_max_iter,
                             int m_line_search) :
                             implicitTimeStepper(m_limbs, m_joints, m_stretch_force, m_bending_force,
                                                 m_twisting_force, m_inertial_force, m_gravity_force,
                                                 m_damping_force, m_floor_contact_force,
                                                 m_force_tol, m_stol, m_max_iter, m_line_search)

{
}

backwardEuler::~backwardEuler() = default;


void backwardEuler::integrator()
{
    pardisoSolver();
    // TODO: move the newton's method stuff here
}


void backwardEuler::newtonMethod() {
    double normf;
    double normf0 = 0;
    bool solved = false;
    iter = 0;

    for (const auto& limb : limbs) limb->updateGuess(0.01);

    while (!solved) {
        prepSystem();
        setZero();

        inertial_force->computeFi();
        inertial_force->computeJi();

        stretching_force->computeFs();
        stretching_force->computeJs();

        bending_force->computeFb();
        bending_force->computeJb();

        twisting_force->computeFt();
        twisting_force->computeJt();

        gravity_force->computeFg();
        gravity_force->computeJg();

        damping_force->computeFd();
        damping_force->computeJd();

        floor_contact_force->computeFfJf();

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

        if (!solved) {
            integrator(); // Solve equations of motion
            if (line_search) lineSearch();

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


void backwardEuler::lineSearch() {
    // store current x
    for (auto& limb : limbs) {
        limb->xold = limb->x;
    }
    for (auto& joint : joints) {
        joint->xold = joint->x;
    }
    // Initialize an interval for optimal learning rate alpha
    double amax = 2;
    double amin = 1e-3;
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
            joint->x = joint->xold;
        }
        for (auto& limb : limbs) {
            limb->x = limb->xold;
            limb->updateNewtonX(dx, offsets[limb_idx], alpha);
            limb_idx++;
        }

        prepSystem();
        setZero();

        // Compute the forces
        inertial_force->computeFi();
        stretching_force->computeFs();
        bending_force->computeFb();
        twisting_force->computeFt();
        gravity_force->computeFg();
        damping_force->computeFd();
        floor_contact_force->computeFf();
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
        limb->x = limb->xold;
    }
    for (auto& joint : joints) {
        joint->x = joint->xold;
    }
    alpha = a;
}


void backwardEuler::stepForwardInTime() {
    newtonMethod();
    updateSystem();
}

