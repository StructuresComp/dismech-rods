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
                      double m_floor_delta, double m_floor_slipTol, double m_mu, double m_dt);
    ~floorContactForce();

    void computeFf(bool fric_off);
    void computeFfJf(bool fric_off);
    void computeFriction(Vector2d curr_node, Vector2d pre_node, double fn);
    void prepFrictionJacobianInput(Vector2d curr_node, Vector2d pre_node, double fn);
    void updateContactStiffness();
    void updateMu(double mu);

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
    double fn;
    double jfn;
    double z;
    double floor_z;
    double contact_stiffness;
    double scale;
    double mu;
    double delta;
    double slipTol;
    double K1;
    double K2;
    double dt;

};
#endif