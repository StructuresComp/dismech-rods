#ifndef SOFT_ROBOTS_H
#define SOFT_ROBOTS_H

#include "elastic_joint.h"
#include "elastic_rod.h"
#include "global_definitions.h"

// Include Controller
#include "controllers/base_controller.h"

class SoftRobots
{
  public:
    SoftRobots();
    ~SoftRobots();

    void addLimb(const Vector3d& start, const Vector3d& end, int num_nodes, double rho,
                 double rod_radius, double youngs_modulus, double poisson_ratio, double mu = 0.0);
    void addLimb(const vector<Vector3d>& nodes, double rho, double rod_radius,
                 double youngs_modulus, double poisson_ratio, double mu = 0.0);

    void createJoint(int limb_idx, int node_idx);
    void addToJoint(int joint_idx, int limb_idx, int node_idx);

    void lockEdge(int limb_idx, int edge_idx);
    void applyInitialVelocities(int limb_idx, const vector<Vector3d>& velocities);
    void applyPositionBC(const Matrix<double, Dynamic, 5>& delta_pos);
    void applyTwistBC(const Matrix<double, Dynamic, 3>& delta_twist);
    void applyCurvatureBC(const Matrix<double, Dynamic, 4>& delta_curvature);

    void setup();

    void addController(const shared_ptr<BaseController>& controller);

    vector<shared_ptr<ElasticRod>> limbs;
    vector<shared_ptr<ElasticJoint>> joints;
    vector<shared_ptr<BaseController>> controllers;

  private:
    int num_limbs = 0;
};

#endif  // SOFT_ROBOTS_H
