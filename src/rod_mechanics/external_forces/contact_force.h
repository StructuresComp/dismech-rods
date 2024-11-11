#ifndef CONTACT_FORCE_H
#define CONTACT_FORCE_H

#include "rod_mechanics/base_force.h"
#include "rod_mechanics/external_forces/collision_detector.h"
#include "rod_mechanics/external_forces/contact_enums.h"
#include "rod_mechanics/external_forces/symbolic_equations.h"

class BaseTimeStepper;

class ContactForce : public BaseForce
{
  public:
    ContactForce(const shared_ptr<SoftRobots>& soft_robots, double col_limit, double delta,
                 double k_scaler, bool friction, double nu, bool self_contact);

    //    void updateContactStiffness();
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;
    void broadPhaseCollisionDetection() const;

    int getNumCollisions() const;
    double getMinDist() const;
    double contact_stiffness;

    unique_ptr<CollisionDetector> col_detector;

  private:
    unique_ptr<SymbolicEquations> sym_eqs;

    double K1;
    double K2;
    double delta;
    double k_scaler;
    bool friction;
    double mu;
    double nu;

    void setupContactVariables(const Vector<int, 8>& contact_id);
    double surface_limit;
    int idx1, idx2, idx3, idx4, idx5, idx6;
    Vector3d x1s, x1e, x2s, x2e, x1s0, x1e0, x2s0, x2e0;

    ConstraintType constraint_type;
    FrictionType friction_type;
    ContactPiecewise contact_type;

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

    void prepContactInput();
    void prepFrictionInput(double dt);
    void computeFriction(double dt);
};

#endif  // CONTACT_FORCE_H
