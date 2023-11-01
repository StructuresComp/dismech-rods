#ifndef ELASTICJOINT_H
#define ELASTICJOINT_H

#include "elasticRod.h"
#include "eigenIncludes.h"

class elasticJoint
{
public:
    // NOTE: probably could move more stuff to private
    elasticJoint(int node, int limb_idx, const vector<shared_ptr<elasticRod>> &limbs);
    int joint_node;
    int joint_limb;

    int ne;

    void updateJoint();
    void updateRods();
    void addToJoint(int node_num, int limb_idx);

    void updateConnectedNodes(int node_num, int limb_idx, bool remove_dof);

    vector<shared_ptr<elasticRod>> limbs;
    Vector3d x;
    Vector3d x0;
    Vector3d x_ls;
    Vector3d u;
    Vector3d u0;
    vector<pair<int, int>> connected_nodes;  // node_number and limb_idx
    vector<pair<int, int>> replaced_nodes;  // node_number and limb_idx

    vector<int> bending_twist_signs;


    double mass;

    int num_bending_combos;

    VectorXd ref_len;
    VectorXd voronoi_len;

    MatrixXd tangents;
    MatrixXd tangents_old;

    vector<Matrix<double, 2, 3>> d1;
    vector<Matrix<double, 2, 3>> d2;
    vector<Matrix<double, 2, 3>> d1_old;
    vector<Matrix<double, 2, 3>> d2_old;
    vector<Matrix<double, 2, 3>> m1;
    vector<Matrix<double, 2, 3>> m2;
    VectorXd ref_twist;

    vector<Vector2i> sgns;
    vector<Vector2i> theta_inds;

    MatrixXd kb;
    MatrixXd kappa;
    MatrixXd kappa_bar;

    VectorXd twist_bar;
    VectorXd ref_twist_old;

    VectorXd edge_len;

    void prepLimbs();
    void prepareForIteration();
    void setup();

private:

    // NOTE: perhaps move these to util.h later?
    void setMass();
    void setReferenceLength();
    void getRefTwist();
    void computeTwistBar();
    void computeEdgeLen();
    void computeTimeParallel();
    void computeKappa();
    void computeTangent();
    void createReferenceDirectors();
    void computeMaterialDirectors();
    static void rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta);
    static void parallelTransport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2);
    static double signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n);
};


#endif
