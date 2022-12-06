#ifndef JOINT_H
#define JOINT_H

#include "elasticRod.h"
#include "../eigenIncludes.h"

class Joint
{
public:
    Joint(int node, int limb_idx, vector<shared_ptr<elasticRod>> &limbs);
    int joint_node;
    int joint_limb;

    int ne;
    double dt;

    void updateJoint();
    void addToJoint(int node_num, int limb_idx);

    void updateConnectedNodes(int node_num, int limb_idx);

    vector<shared_ptr<elasticRod>>& limbs;
    Vector3d x;
    Vector3d x0;
    Vector3d u;
    vector<pair<int, int>> connected_nodes;  // node_number and limb_idx

    void setup();

    double mass;
    void setMass();

    int num_bending_combos;

    VectorXd ref_len;
    VectorXd voronoi_len;
    void setReferenceLength();

    MatrixXd tangents;
    MatrixXd tangents_old;
    void computeTangent();

    void getRefDirectors();
    void computeMaterialDirectors();

    MatrixXd d1;
    MatrixXd d2;
    MatrixXd d1_old;
    MatrixXd d2_old;
    MatrixXd m1;
    MatrixXd m2;
    VectorXd ref_twist;

    MatrixXd kb;
    MatrixXd kappa;
    MatrixXd kappaBar;
    void computeKappa();

    VectorXd undeformed_twist;
    VectorXd ref_twist_old;
    void rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta);
    void parallelTransport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2);
    double signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n);

    void getRefTwist();

    VectorXd edge_len;
    void computeEdgeLen();

    void prepareForIteration();
    void computeTimeParallel();

    void updateTimeStep();

private:
};


#endif
