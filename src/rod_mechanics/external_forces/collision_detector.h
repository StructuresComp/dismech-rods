#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "rod_mechanics/external_forces/contact_enums.h"
#include "rod_mechanics/soft_robots.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/narrowphase/collision.h>

class CollisionDetector
{
  public:
    explicit CollisionDetector(const std::shared_ptr<SoftRobots>& soft_robots, double col_limit,
                               double delta, bool self_contact);

    struct LimbEdgeInfo
    {
        LimbEdgeInfo(int limb_id, int edge_id) : limb_id(limb_id), edge_id(edge_id) {
        }

        int limb_id;
        int edge_id;
    };

    struct ContactPair
    {
        ContactPair(LimbEdgeInfo* c1, LimbEdgeInfo* c2) : c1(c1), c2(c2) {
        }

        LimbEdgeInfo* c1;
        LimbEdgeInfo* c2;
    };

    int num_collisions;
    double min_dist;
    std::vector<ContactPair> broad_phase_collision_set;
    void broadPhaseCollisionDetection();
    void narrowPhaseCollisionDetection();
    std::vector<Eigen::Vector<int, 8>> contact_ids;

  private:
    double delta;
    double col_limit;
    bool self_contact;
    std::shared_ptr<SoftRobots> soft_robots;

    std::vector<std::vector<LimbEdgeInfo>> limb_edge_ids;
    std::vector<std::vector<fcl::CollisionObjectf*>> cylinders;
    void prepCylinders();

    Eigen::Vector3f a;
    Eigen::Matrix3f rot_mat;
    void getRotMat(Eigen::Vector3f& b);
    std::vector<fcl::BroadPhaseCollisionManagerf*> collision_managers;

    static bool fixBound(double& x);
    void lumelskyMinDist(int& idx1, int& idx2, int& idx3, int& idx4, int& idx5, int& idx6,
                         double& dist, ConstraintType& constraint_type);
};

#endif  // COLLISION_DETECTOR_H
