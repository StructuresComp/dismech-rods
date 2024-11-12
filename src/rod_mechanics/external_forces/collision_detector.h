#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "global_definitions.h"
#include "rod_mechanics/external_forces/contact_enums.h"

class SoftRobots;

namespace fcl
{
template <typename S>
class CollisionObject;
using CollisionObjectf = CollisionObject<float>;

template <typename S>
class BroadPhaseCollisionManager;
using BroadPhaseCollisionManagerf = BroadPhaseCollisionManager<float>;

template <typename S>
class DynamicAABBTreeCollisionManager;
using DynamicAABBTreeCollisionManagerf = DynamicAABBTreeCollisionManager<float>;
}  // namespace fcl

class CollisionDetector
{
  public:
    explicit CollisionDetector(const std::shared_ptr<SoftRobots>& soft_robots, double col_limit,
                               double delta, bool self_contact);
    ~CollisionDetector();

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
