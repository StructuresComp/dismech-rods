#ifndef CONTACTPOTENTIALIMC_H
#define CONTACTPOTENTIALIMC_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"
#include "collisionDetector.h"
#include "symbolicEquations.h"
#include "contactEnums.h"


class contactPotentialIMC
{
public:
    contactPotentialIMC(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper,
                        shared_ptr<collisionDetector> m_col_detector,
                        double m_delta, double m_k_scaler, double m_mu, double m_nu);

    void updateContactStiffness();
    void computeFc();
    void computeFcJc();
    double contact_stiffness;

private:
    shared_ptr<elasticRod> rod = nullptr;
    shared_ptr<timeStepper> stepper = nullptr;
    shared_ptr<collisionDetector> col_detector = nullptr;
    unique_ptr<symbolicEquations> sym_eqs = nullptr;
    double K1;
    double K2;
    double h2;
    double delta;
    double k_scaler;
    bool friction;
    double mu;
    double nu;
    double scale;
    int idx1;
    int idx2;
    int idx3;
    int idx4;
    ConstraintType constraint_type;
    FrictionType friction_type;
    ContactPiecewise contact_type;

    // Index helper
    vector<double> di{0, 1, 2, 4, 5, 6};

    Vector<double, 8> p2p_input;
    Vector<double, 11> e2p_input;
    Vector<double, 14> e2e_input;

    Vector<double, 6> p2p_gradient;
    Vector<double, 9> e2p_gradient;
    Vector<double, 12> e2e_gradient;

    Matrix<double, 6, 6> p2p_hessian;
    Matrix<double, 9, 9> e2p_hessian;
    Matrix<double, 12, 12> e2e_hessian;

    Vector<double, 39> friction_input;
    Vector<double, 12> contact_gradient;
    Vector<double, 12> friction_forces;
    Matrix<double, 12, 12> contact_hessian;
    Matrix<double, 12, 12> friction_partials_dfr_dx;
    Matrix<double, 12, 12> friction_partials_dfr_dfc;
    Matrix<double, 12, 12> friction_jacobian;
    Matrix<double, 3, 12> friction_zero_matrix;

    void prepContactInput();
    void prepFrictionInput();
    void computeFriction();
};

#endif