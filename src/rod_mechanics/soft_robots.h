#ifndef SOFT_ROBOTS_H
#define SOFT_ROBOTS_H

#include "global_definitions.h"

class ElasticRod;
class ElasticJoint;
class BaseController;

class SoftRobots
{
  public:
    SoftRobots();
    ~SoftRobots();

    void addLimb(const Vec3& start, const Vec3& end, int num_nodes, double rho, double rod_radius,
                 double youngs_modulus, double poisson_ratio, double mu = 0.0,
                 uint16_t col_group = 0xFFFF);
    void addLimb(const std::vector<Vec3>& nodes, double rho, double rod_radius,
                 double youngs_modulus, double poisson_ratio, double mu = 0.0,
                 uint16_t col_group = 0xFFFF);

    void createJoint(int limb_idx, int node_idx);
    void addToJoint(int joint_idx, int limb_idx, int node_idx);

    void lockEdge(int limb_idx, int edge_idx);
    void applyInitialVelocities(int limb_idx, const std::vector<Vec3>& velocities);
    void applyPositionBC(const MatXN<5>& positions);
    void applyDeltaPositionBC(const MatXN<5>& delta_positions);
    void applyThetaBC(const MatXN<3>& thetas);
    void applyDeltaThetaBC(const MatXN<3>& delta_thetas);
    void applyCurvatureBC(const MatXN<4>& curvatures);
    void applyDeltaCurvatureBC(const MatXN<4>& delta_curvatures);

    void setup();

    void addController(const std::shared_ptr<BaseController>& controller);

    std::vector<std::shared_ptr<ElasticRod>> limbs;
    std::vector<std::shared_ptr<ElasticJoint>> joints;
    std::vector<std::shared_ptr<BaseController>> controllers;

  private:
    int num_limbs = 0;
};

#endif  // SOFT_ROBOTS_H
