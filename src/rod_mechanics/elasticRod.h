#ifndef ELASTICROD_H
#define ELASTICROD_H

#include "eigenIncludes.h"

class elasticRod
{
    // NOTE: probably could move more stuff to private
    public:
    /**
     * @brief Constructor for initializing a straight rod as defined by a start and end point.
     *
     * @param limb_idx The limb id of the rod. Should be a +1 increment of the previous rod.
     *                 Other inputs will lead to undefined behavior.
     * @param start The start point of the rod.
     * @param end The end point of the rod.
     * @param num_nodes The number of nodes the rod will be discretized to.
     * @param rho Density of the rod [kg/m^3].
     * @param rod_radius Radius of the rod [m].
     * @param youngs_modulus Young's modulus of the rod [N/m^2].
     * @param poisson_ratio Poisson's ratio of the rod.
     * @param mu Friction coefficient of the rod.
     */
    elasticRod(int limb_idx, const Vector3d& start, const Vector3d& end, int num_nodes,
               double rho, double rod_radius, double youngs_modulus, double poisson_ratio, double mu);

    /**
     * @brief Constructor for initializing an arbitrarily shaped rod as by a list of nodes.
     *
     * @param limb_idx The limb id of the rod. Should be a +1 increment of the previous rod.
     *                 Other inputs will lead to undefined behavior.
     * @param nodes A list of consecutive nodes of the rod.
     * @param rho Density of the rod [kg/m^3].
     * @param rod_radius Radius of the rod [m].
     * @param youngs_modulus Young's modulus of the rod [N/m^2].
     * @param poisson_ratio Poisson's ratio of the rod.
     * @param mu Friction coefficient of the rod.
     */
    elasticRod(int limb_idx, const vector<Vector3d>& nodes, double rho, double rod_radius,
               double youngs_modulus, double poisson_ratio, double mu);

    /**
     * @brief Default deconstructor.
     */
    ~elasticRod();

    /**
     * @brief Updates all necessary discrete values and frames for edges
     * in preparation of next time step.
     *
     * @details Updates discrete tangents, reference frames, material frames,
     * reference twists, edge lengths, and curvatures.
     */
    void prepareForIteration();

    /**
     * @brief Performs a Newton update iteration.
     *
     * @param dx The update vector provided by the time stepper and solver.
     * @param offset A constant offset for the rod's DOF index in dx with
     * respect to the global system.
     * @param alpha A Newton update coefficient.
     *
     * @return The max non-theta dx update.
     */
    double updateNewtonX(double *dx, int offset, double alpha=1.0);

    /**
     * @brief Updates the current guess at the start of a Newton loop.
     *
     * @details Updates all unconstrained DOFs using
     *
     *     x = x0 + weight + u * dt
     *
     * Generally, performance seems best when the guess is close to x0,
     * especially when friction is involved.
     *
     * @param weight The weight of the velocity update shown above.
     * @param dt The time step.
     */
    void updateGuess(double weight, double dt);

    /**
     * @brief Enables 2D simulation along x-z plane.
     *
     * @details Assumes all y and theta related DOFs to be constrained.
     */
    void enable2DSim() const;

    /**
     * @brief The limb id of the rod.
     */
    int limb_idx;

    /**
     * @brief Get the a (3,) position vector of the kth node from current time step.
     *
     * @param k Index of the node.
     *
     * @return The (3,) position vector.
     */
    Vector3d getVertex(int k);

    /**
     * @brief Get the a (3,) position vector of the kth node from previous time step.
     *
     * @param k Index of the node.
     *
     * @return The (3,) position vector.
     */
    Vector3d getPreVertex(int k);

    /**
     * @brief Get the a (3,) velocity vector of the kth node from current time step.
     *
     * @param k Index of the node.
     *
     * @return The (3,) velocity vector.
     */
    Vector3d getVelocity(int k);

    /**
     * @brief Density [kg/m^3]
     */
    double rho;

    /**
     * @brief Cross-sectional radius of the rod [m].
     *
     * @details Currently, assumes constant radius.
     */
    double rod_radius;

    /**
     * @brief Cross-sectional area of the rod [m^2].
     */
    double cross_sectional_area;

    /**
     * @brief Friction coefficient.
     */
    double mu;

    /**
     * @brief Total relaxed length of the rod [m].
     */
    double rod_length;

    /**
     * @brief Young's modulus.
     */
    double youngM;

    /**
     * @brief Shear modulus.
     *
     * @see Computed in computeElasticStiffness().
     */
    double shearM;

    /**
     * @brief Poisson ratio.
     */
    double poisson_ratio;

    /**
     * @brief Stretching stiffness.
     *
     * @see Computed in computeElasticStiffness().
     */
    double EA;

    /**
     * @brief Bending stiffness.
     *
     * @see Computed in computeElasticStiffness().
     */
    double EI;

    /**
     * @brief Twisting stiffness.
     *
     * @see Computed in computeElasticStiffness().
     */
    double GJ;

    /**
     * @brief Number of vertices (nodes).
     */
    int nv;

    /**
     * @brief Number of edges.
     *
     * @details Number of edges is always one less than the
     * number of nodes:
     *
     *     ne = nv-1
     */
    int ne;

    /**
     * @brief Number of degrees of freedom (DOF).
     *
     * @details Number of DOFs is always
     *
     *     ndof = 3*nv + ne
     */
    int ndof;

    /**
     * @brief Number of constrained DOF.
     */
    int ncons;

    /**
     * @brief Number of unconstrained DOF.
     */
    int uncons;

    /**
     * @brief DOF vector from previous time step.
     *
     * @details Vector of size (ndof, 1).
     */
    VectorXd x0;

    /**
     * @brief DOF vector during and after time step.
     *
     * @details Vector of size (ndof, 1).
     */
    VectorXd x;

    /**
     * @brief DOF vector used to save state during line search.
     *
     * @details Vector of size (ndof, 1).
     */
    VectorXd x_ls;

    /**
     * @brief Velocity vector from previous time step.
     *
     * @details Vector of size (ndof, 1).
     */
    VectorXd u0;

    /**
     * @brief Velocity vector during and after time step.
     *
     * @details Vector of size (ndof, 1).
     */
    VectorXd u;

    /**
     * @brief Vector of current edge lengths [m].
     *
     * @see Computed in computeEdgeLen().
     */
    VectorXd edge_len;

    /**
     * @brief Reference lengths of the discrete edges [m].
     *
     * @details Computed only once during initialization.
     * Starting configuration is assumed to be resting configuration.
     *
     * @see Computed in setReferenceLength().
     */
    VectorXd ref_len;

    /**
     * @brief Voronoi lengths of each node [m].
     *
     * @details Computed only once during initialization.
     * Computed from reference lengths.
     *
     * @see Computed in setReferenceLength().
     */
    VectorXd voronoi_len;

    /**
     * @brief Curvature binormals of each discrete edge.
     *
     * @details Computed during every iteration / time step to compute
     * the discrete curvatures necessary to compute bending energy.
     *
     * @see Computed in computeKappa().
     */
    MatrixXd kb;

    /**
     * @brief d1 axes of the reference frame during current iteration / time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTimeParallel().
     * @see Used in getRefTwist().
     */
    MatrixXd d1;

    /**
     * @brief d2 axes of the reference frame during current iteration / time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTimeParallel().
     * @see Used in getRefTwist().
     */
    MatrixXd d2;

    /**
     * @brief d1 axes of the reference frame from last time step.
     *
     * @details Matrix of size (ne, 3). Updated by time stepper classes when completing a time step.
     *
     * @see Used in computeTimeParallel().
     */
    MatrixXd d1_old;

    /**
     * @brief d2 axes of the reference frame from last time step.
     *
     * @details Matrix of size (ne, 3). Updated by time stepper classes when completing a time step.
     */
    MatrixXd d2_old;

    /**
     * @brief m1 axes of the material frame during current iteration / time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeMaterialDirector().
     * @see Used in computeKappa() to compute discrete curvatures.
     */
    MatrixXd m1;

    /**
     * @brief m2 axes of the material frame during current iteration / time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeMaterialDirector().
     * @see Used in computeKappa() to compute discrete curvatures.
     */
    MatrixXd m2;

    /**
     * @brief Edge tangents during the current iteration / time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTangent().
     * @see Used in computeTimeParallel().
     */
    MatrixXd tangent;

    /**
     * @brief Edge tangents from the previous time step.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Used in computeTimeParallel().
     */
    MatrixXd tangent_old;

    /**
     * @brief Reference twists during the current iteration / time step.
     *
     * @details Vector of size (ne, 1).
     *
     * @see Updated in getRefTwist().
     * @see Used in computeTwistBar().
     */
    VectorXd ref_twist;

    /**
     * @brief Reference twists from the previous time step.
     *
     * @details Vector of size (ne, 1).
     */
    VectorXd ref_twist_old;

    /**
     * @brief The twists in the resting configuration.
     *
     * @details Vector of size (ne, 1).
     *
     * @see Updated in computeTwistBar().
     */
    VectorXd twist_bar;

    /**
     * @brief Lumped mass array.
     *
     * @details Vector of size (ndof, 1). Stores the lumped masses of each DOF.
     *
     * @see Updated in setMass().
     *
     */
    VectorXd mass_array;

    /**
     * @brief Discrete curvatures of the inner nodes.
     *
     * @details Matrix of size (nv, 2). Note that the first and outer values
     * are not used since curvatures only apply to nodes with two edges attached.
     *
     * @see Updated in computeKappa().
     */
    MatrixXd kappa;

    /**
     * @brief Discrete curvatures of the inner nodes in the resting configuration.
     *
     * @details Matrix of size (nv, 2). Note that the first and outer values
     * are not used since curvatures only apply to nodes with two edges attached.
     * Is assumed to the curvatures computed from the initial starting configuration.
     */
    MatrixXd kappa_bar;

    /**
     * @brief Array storing whether or not a certain DOF is constrained.
     *
     * @details Size of ndof. Value of 1 if constrained and 0 if unconstrained.
     */
    int *isConstrained;

    /**
     * @brief Getter function for seeing if a certain DOF is constrained.
     *
     * @param k Index of the queried DOF.
     *
     * @return Value of 1 if constrained and 0 if unconstrained.
     */
    int getIfConstrained(int k) const;

    /**
     * @brief Array that maps unconstrained to full indices.
     *
     * @details Size of uncons. Maps unconstrained DOF (by array location)
     * to their respective index in x vector. The index of the mapping is stored.
     *
     * @see Gets updated in updateMap() and in setup().
     * @see The inverse mapping is fullToUnconsMap.
     */
    int *unconstrainedMap;

    /**
     * @brief Array that maps full to unconstrained indices.
     *
     * @details Size of ndof. Maps all DOF from x vector (by array location) to
     * their respective in unconstrained vector. The index of the mapping is stored.
     * If a certain DOF is constrained, then the value at that array location
     * is undefined.
     *
     * @see Gets updated in updateMap() and in setup().
     * @see The inverse mapping is unconstrainedMap.
     */
    int *fullToUnconsMap;

    /**
     * @brief Updates the constraint map according to current values of
     * isConstrained and isDOFJoint.
     *
     * @details Must be called when new boundary conditions are created
     * or if boundary conditions are released. If the value of a
     * preexisting boundary condition is updated, this function does not
     * need to be called.
     */
    void updateMap();

    // TODO: perhaps split this up into a createJoint() and attachToJoint() functions.
    /**
     * @brief Converts a node on the rod to a joint node.
     *
     * @details This function should NOT be called by the user itself.
     * The elasticJoint class will handle calling this function.
     *
     * There are two types of applications to this function.
     *
     * 1) The node is being converted to a joint node that does not
     * previously exist. To create a joint node for the first time,
     * attach_to_joint must be set to False. In this setting, joint_node
     * and joint_limb are ignored.
     *
     * 2) The node is being attached to a preexisting joint node. As of
     * right now, only the first and last nodes can be converted in this
     * fashion. To attach this node to preexisting joint node,
     * attach_to_joint must be set to True and joint_node and joint_limb
     * must be set to the desired preexisting joint node.
     *
     * @param node_num The index for the node to be converted.
     * @param attach_to_joint Whether or not this node is being attached
     * to a preexisting joint.
     * @param joint_node The joint node id of the preexisting joint.
     * @param joint_limb The joint limb id of the preexisting joint.
     */
    void addJoint(int node_num, bool attach_to_joint, int joint_node, int joint_limb);

    /**
     * @brief Array indicating whether or not a certain DOF is a joint node.
     *
     * @details Size of ndof. Can be one of three possible values:
     * 0: DOF is not a joint affiliated DOF.
     * 1: DOF is a joint affiliated DOF and was attached to a preexisting joint.
     * 2: DOF is a joint affiliated DOF and was used to create a joint
     * node for the first time.
     *
     * @see Updated in addJoint().
     */
    int *isDOFJoint;

    /**
     * @brief Array indicating whether or not a certain node is a joint node.
     *
     * @details Size of nv. Follows the same rules as isDOFJoint.
     *
     * @see Updated in addJoint().
     */
    int *isNodeJoint;

    /**
     * @brief Array indicating whether or not a certain edge is adjacent
     * to a joint node.
     *
     * @details Size of ne. Follows the same rules as isDOFJoint.
     *
     * @see Updated in addJoint().
     */
    int *isEdgeJoint;

    /**
     * @brief Vector of joint node and limb ids.
     *
     * @details Stores a list of pairs of joint node and joint limb ids.
     * Used to reroute forces and Jacobians to their proper locations in
     * the global force vector / Jacobian matrix for nodes that were attached
     * to preexisting joint nodes.
     * Currently of size nv.
     *
     * @see Updated in addJoint().
     */
    vector<pair<int, int>> joint_ids;

    void setVertexBoundaryCondition(Vector3d position, int k);
    void setThetaBoundaryCondition(double desired_theta, int k);
    void freeVertexBoundaryCondition(int k);
    Vector3d getTangent(int k);
    double getTheta(int k);

private:
    void setupMap();

    // TODO: perhaps move these to util.h later as most can be defined as staticmethods?
    void setup(const vector<Vector3d>& nodes);
    void computeElasticStiffness();
    void setMass();
    void setReferenceLength();
    void computeTimeParallel();
    void computeTangent();
    void computeSpaceParallel();
    void computeMaterialDirector();
    void computeKappa();
    void computeTwistBar();
    void computeEdgeLen();
    void getRefTwist();
    static void parallelTransport(const Vector3d &d1_1,const Vector3d &t1,const Vector3d &t2,Vector3d &d1_2);
    static void rotateAxisAngle(Vector3d &v, const Vector3d &z, const double &theta);
    static double signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n);
};

#endif
