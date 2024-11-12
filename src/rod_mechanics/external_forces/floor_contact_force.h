#ifndef FLOOR_CONTACT_FORCE_H
#define FLOOR_CONTACT_FORCE_H

#include "rod_mechanics/base_force.h"
#include "symbolic_equations.h"

class BaseTimeStepper;

class FloorContactForce : public BaseForce
{
  public:
    FloorContactForce(const std::shared_ptr<SoftRobots>& soft_robots, double floor_delta,
                      double floor_slipTol, double floor_z, double floor_mu = 0.0);
    ~FloorContactForce() override;

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

    double min_dist;
    double floor_z;
    int num_contacts;

  private:
    void computeFriction(const Vec2& curr_node, const Vec2& pre_node, double fn, double mu,
                         double dt);
    void prepFrictionJacobianInput(const Vec2& curr_node, const Vec2& pre_node, double fn,
                                   double mu, double dt);

    std::shared_ptr<SymbolicEquations> sym_eqs;
    Vec2 contact_input;
    Vec<8> fric_jacobian_input;
    Mat2 friction_partials_dfr_dx;
    Vec2 friction_partials_dfr_dfn;
    Vec2 ffr;
    int fric_jaco_type;
    double contact_stiffness;
    double floor_mu;
    double delta;
    double slipTol;
    double K1;
    double K2;
};

#endif  // FLOOR_CONTACT_FORCE_H
