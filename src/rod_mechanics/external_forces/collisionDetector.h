#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <fcl/narrowphase/collision.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include "rod_mechanics/softRobots.h"
#include "rod_mechanics/external_forces/contactEnums.h"


class collisionDetector
{
public:
    explicit collisionDetector(const shared_ptr<softRobots>& soft_robots, double col_limit,
                               double delta, bool self_contact);
    struct limb_edge_info {
        limb_edge_info(int limb_id, int edge_id):
                       limb_id(limb_id), edge_id(edge_id)
        {}
        int limb_id;
        int edge_id;
    };

    struct contact_pair {
        contact_pair(limb_edge_info* c1, limb_edge_info* c2) :
                     c1(c1), c2(c2)
        {}
        limb_edge_info* c1;
        limb_edge_info* c2;
    };

    int num_collisions;
    double min_dist;
    vector<contact_pair> broad_phase_collision_set;
    void broadPhaseCollisionDetection();
    void narrowPhaseCollisionDetection();
    vector<Eigen::Vector<int, 8>> contact_ids;

private:
    double delta;
    double col_limit;
    bool self_contact;
    shared_ptr<softRobots> soft_robots;

    vector<vector<limb_edge_info>> limb_edge_ids;
    vector<vector<fcl::CollisionObjectf*>> cylinders;
    void prepCylinders();

    Eigen::Vector3f a;
    Eigen::Matrix3f rot_mat;
    void getRotMat(Eigen::Vector3f& b);
    vector<fcl::BroadPhaseCollisionManagerf*> collision_managers;

    static bool fixBound(double& x);
    void lumelskyMinDist(int& idx1, int& idx2, int& idx3, int& idx4, int& idx5,
                          int& idx6, double& dist, ConstraintType& constraint_type);
};


#endif

