#ifndef FLOORCONTACT_H
#define FLOORCONTACT_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"
#include "symbolicEquations.h"

class floorContactForce
{
public:
    floorContactForce(vector<shared_ptr<elasticRod>> m_limbs, shared_ptr<timeStepper> m_stepper,
                      double m_floor_delta, double m_floor_slipTol, double m_mu, double m_dt, double m_floor_z);
    ~floorContactForce();

    void computeFf();
    void computeFfJf();
    void computeFriction(const Vector2d& curr_node, const Vector2d& pre_node, double fn);
    void prepFrictionJacobianInput(const Vector2d& curr_node, const Vector2d& pre_node, double fn);
    void updateMu(double mu);

    double min_dist;
private:
    vector<shared_ptr<elasticRod>> limbs;
    shared_ptr<timeStepper> stepper;
    shared_ptr<symbolicEquations> sym_eqs;
    Vector<double, 2> contact_input;
    Vector<double, 8> fric_jacobian_input;
    Matrix<double, 2, 2> friction_partials_dfr_dx;
    Vector<double, 2> friction_partials_dfr_dfn;
    Vector<double, 2> ffr;
    int fric_jaco_type;
    double z;
    double contact_stiffness;
    double scale;
    double mu;
    double delta;
    double slipTol;
    double K1;
    double K2;
    double dt;
    double floor_z;

};
#endif