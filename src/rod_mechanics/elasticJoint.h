#ifndef ELASTICJOINT_H
#define ELASTICJOINT_H

#include "elasticRod.h"
#include "../eigenIncludes.h"

class elasticJoint
{
public:
    elasticJoint(int node, int limb_idx, const vector<shared_ptr<elasticRod>> &limbs);
    int joint_node;
    int joint_limb;

    int ne;
    double dt;

    void updateJoint();
    void updateRods();
    void addToJoint(int node_num, int limb_idx);

    void updateConnectedNodes(int node_num, int limb_idx, bool remove_dof);

    vector<shared_ptr<elasticRod>> limbs;
    Vector3d x;
    Vector3d x0;
    Vector3d xold;
    Vector3d u;
    vector<pair<int, int>> connected_nodes;  // node_number and limb_idx
    vector<pair<int, int>> replaced_nodes;  // node_number and limb_idx

    vector<int> bending_twist_signs;

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

    void createReferenceDirectors();
    void computeMaterialDirectors();

    vector<Matrix<double, 2, 3>> d1;
    vector<Matrix<double, 2, 3>> d2;
    vector<Matrix<double, 2, 3>> d1_old;
    vector<Matrix<double, 2, 3>> d2_old;
    vector<Matrix<double, 2, 3>> m1;
    vector<Matrix<double, 2, 3>> m2;
    VectorXd ref_twist;

    MatrixXd kb;
    MatrixXd kappa;
    MatrixXd kappaBar;
    void computeKappa();

    VectorXd twistBar;
    VectorXd ref_twist_old;
    void rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta);
    void parallelTransport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2);
    double signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n);

    void getRefTwist();
    void computeTwistBar();

    VectorXd edge_len;
    void computeEdgeLen();

    void prepLimbs();
    void prepareForIteration();
    void computeTimeParallel();

    void updateTimeStep();

private:
};


#endif
