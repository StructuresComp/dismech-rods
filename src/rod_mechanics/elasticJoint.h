#ifndef ELASTICJOINT_H
#define ELASTICJOINT_H

#include "elasticRod.h"
#include "eigenIncludes.h"

class elasticJoint
{
    // NOTE: probably could move more stuff to private
public:
    /**
     * @brief Constructor for initializing an elastic joint.
     *
     * @param node The node on the elastic rod that will be converted to a joint.
     * @param limb_idx The limb id of the rod.
     * @param limbs Container of limbs.
     */
    elasticJoint(int node, int limb_idx, const vector<shared_ptr<elasticRod>> &limbs);

    /**
     * @brief The node id of the joint.
     */
    int joint_node;

    /**
     * @brief The limb id of the joint.
     */
    int joint_limb;

    /**
     * @brief Total number of edges that connect to this joint.
     */
    int ne;

    /**
     * @brief Adds a connection to the joint
     *
     * @details When we connect a node to a joint, that node is actually
     * "replaced" with the joint node. Therefore, we assume that nodes
     * that are connected to a joint have positional overlap.
     *
     * @param node_num The id of the node that is being replaced by the joint node.
     * @param limb_idx The limb id of the replaced node.
     */
    void addToJoint(int node_num, int limb_idx);

    /**
     * @brief Updates the positions and velocities of the joint node.
     *
     * @details All positions and velocities are computed within elasticRod.
     * Therefore, before we compute anything within the elasticJoint class,
     * we need to copy over the correct positions and velocities.
     */
    void updateJoint();

    /**
     * @brief Updates the positions of the "replaced" nodes in other limbs.
     *
     * @details Recall that when attaching a rod to an existing joint, the
     * overlapping node is "replaced" with the joint node. Behind the scenes,
     * this node actually still coexists with the joint node. Therefore,
     * for certain computations within elasticRod not to be incorrect, we need
     * to update the position of the "replaced" node to match the joint node.
     */
    void updateRods();

    /**
     * @brief A helper function to store miscellaneous info when the joint is updated. 
     * 
     * @param node_num The id of the node that is being replaced by the joint node.
     * @param limb_idx The limb id of the replaced node.
     * @param remove_dof Whether the node is being attached to a preexisting joint.
     * For example, this value should only be False for the very first node being
     * converted to a joint node at initialization.
     */
    void updateConnectedNodes(int node_num, int limb_idx, bool remove_dof);

    /**
     * @brief The stored limbs of the simulation.
     */
    vector<shared_ptr<elasticRod>> limbs;

    /**
     * @brief The position of the joint node from the previous timestep.
     */
    Vector3d x0;
    
    /**
     * @brief The position of the joint node during and after the current timestep.
     */
    Vector3d x;
    
    /**
     * @brief Position vector used to save state during line search. 
     */
    Vector3d x_ls;
    
    /**
     * @brief Velocity vector from the previous timestep.
     */
    Vector3d u0;
    
    /**
     * @brief Velocity vector during and after the current timestep.
     */
    Vector3d u;

    /**
     * @brief Container storing the ids of all nodes that have an edge
     * connection to the joint node.
     *
     * @details First value is the node id
     *          Second value is the limb id
     */
    vector<pair<int, int>> connected_nodes;

    /**
     * @brief Container storing the ids of all the replaced nodes.
     *
     * @details First value is the node id
     *          Second value is the limb id
     */
    vector<pair<int, int>> replaced_nodes;

    /**
     * @brief The direction of the tangent of each edge connected to
     * the joint node, used to compute bending and twisting force signs.
     *
     * @details For each edge connected to the joint node, a value
     * is stored to keep track whether the tangent of the node goes
     * toward the joint node or out of the joint node. This direction
     * of the tangent is determined when each rod is initialized.
     * A value of 1: the tangent is going into the joint node.
     * A value of -1: the tangent is going out from the joint node.
     *
     * @see Used in elasticBendingForce.cpp and elasticTwistingForce.cpp
     */
    vector<int> bending_twist_signs;

    /**
     * @brief The number of possible non-repeating edge combinations
     * connected to this joint.
     */
    int num_bending_combos;

    /**
     * @brief The sign of the bending and twisting forces.
     *
     * @details Depending on the direction the edges are initialized,
     * bending and twisting forces must sometimes be multiplied by -1.
     * For elastic rods without joints, this is never an issue, but when
     * two rods are connected in a way that the direction of the tangents
     * go towards each-other rather than along the same direction, then
     * at least one of the forces must be multiplied by -1.
     * This vector holds a value of either 1 or -1.
     *
     * @see Computed using bending_twist_signs
     * @see Used in elasticBendingForce.cpp and elasticTwistingForce.cpp
     */
    vector<Vector2i> sgns;

    /**
     * @brief Indices for the theta DOFs for each edge combination.
     *
     * @details Vector is size num_bending_combos and contains the
     * indices of first and second edge's theta DOF, respectively.
     */
    vector<Vector2i> theta_inds;

    /**
     * @brief Reference lengths of the discrete edges connected to the joint.
     */
    VectorXd ref_len;

    /**
     * @brief Voronoi lengths of the discrete edges connected to the joint.
     */
    VectorXd voronoi_len;

    /**
     * @brief The mass of the joint.
     */
    double mass;

    MatrixXd tangents;
    MatrixXd tangents_old;

    vector<Matrix<double, 2, 3>> d1;
    vector<Matrix<double, 2, 3>> d2;
    vector<Matrix<double, 2, 3>> d1_old;
    vector<Matrix<double, 2, 3>> d2_old;
    vector<Matrix<double, 2, 3>> m1;
    vector<Matrix<double, 2, 3>> m2;
    VectorXd ref_twist;

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

    // TODO: perhaps move these to util.h later? The logic is largely repeated in elasticRod.h
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
