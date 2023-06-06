#ifndef FLOORCONTACT_H
#define FLOORCONTACT_H

#include "baseForce.h"
#include "symbolicEquations.h"

class baseTimeStepper;

class floorContactForce : public baseForce
{
public:
    floorContactForce(const vector<shared_ptr<elasticRod>>& m_limbs, const vector<shared_ptr<elasticJoint>>& m_joints,
                      double m_floor_delta, double m_floor_slipTol, double m_mu, double m_floor_z);
    ~floorContactForce() override;

    void computeFf(double dt);
    void computeFfJf(double dt);
    void computeFriction(const Vector2d& curr_node, const Vector2d& pre_node, double fn, double dt);
    void prepFrictionJacobianInput(const Vector2d& curr_node, const Vector2d& pre_node, double fn, double dt);
    void updateMu(double mu);

    double min_dist;

    void change_slip_tol(double scale);
    void reset_slip_tol();

private:
    shared_ptr<symbolicEquations> sym_eqs;
    Vector<double, 2> contact_input;
    Vector<double, 8> fric_jacobian_input;
    Matrix<double, 2, 2> friction_partials_dfr_dx;
    Vector<double, 2> friction_partials_dfr_dfn;
    Vector<double, 2> ffr;
    int fric_jaco_type;
    double contact_stiffness;
    double mu;
    double delta;
    double slipTol;
    double orig_slip_tol;
    double K1;
    double K2;
    double floor_z;
};
#endif