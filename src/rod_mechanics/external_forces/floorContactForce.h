#ifndef FLOORCONTACT_H
#define FLOORCONTACT_H

#include "symbolicEquations.h"
#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class floorContactForce : public baseForce
{
public:
    floorContactForce(const shared_ptr<softRobots>& soft_robots, double floor_delta,
                      double floor_slipTol, double floor_z, double floor_mu=0.0);
    ~floorContactForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

    double min_dist;
    double floor_z;
    int num_contacts;

    void change_slip_tol(double scale);
    void reset_slip_tol();

private:
    void computeFriction(const Vector2d& curr_node, const Vector2d& pre_node, double fn, double mu, double dt);
    void prepFrictionJacobianInput(const Vector2d& curr_node, const Vector2d& pre_node, double fn, double mu, double dt);

    shared_ptr<symbolicEquations> sym_eqs;
    Vector<double, 2> contact_input;
    Vector<double, 8> fric_jacobian_input;
    Matrix<double, 2, 2> friction_partials_dfr_dx;
    Vector<double, 2> friction_partials_dfr_dfn;
    Vector<double, 2> ffr;
    int fric_jaco_type;
    double contact_stiffness;
    double floor_mu;
    double delta;
    double slipTol;
    double orig_slip_tol;
    double K1;
    double K2;
};
#endif