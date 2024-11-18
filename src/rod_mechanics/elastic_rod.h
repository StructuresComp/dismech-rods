#ifndef ELASTIC_ROD_H
#define ELASTIC_ROD_H

#include "global_definitions.h"

class ElasticRod
{
    // NOTE: probably could move more stuff to private
  public:
    /**
     * @brief Constructor for initializing a straight rod as defined by a start
     * and end point.
     *
     * @param limb_idx The limb id of the rod. Should be a +1 increment of the
     * previous rod. Other inputs will lead to undefined behavior.
     * @param start The start point of the rod.
     * @param end The end point of the rod.
     * @param num_nodes The number of nodes the rod will be discretized to.
     * @param rho Density of the rod [kg/m^3].
     * @param rod_radius Radius of the rod [m].
     * @param youngs_modulus Young's modulus of the rod [N/m^2].
     * @param poisson_ratio Poisson's ratio of the rod.
     * @param mu Friction coefficient of the rod.
     */
    ElasticRod(int limb_idx, const Vec3& start, const Vec3& end, int num_nodes, double rho,
               double rod_radius, double youngs_modulus, double poisson_ratio, double mu);

    /**
     * @brief Constructor for initializing an arbitrarily shaped rod as by a
     * list of nodes.
     *
     * @param limb_idx The limb id of the rod. Should be a +1 increment of the
     * previous rod. Other inputs will lead to undefined behavior.
     * @param nodes A list of consecutive nodes of the rod.
     * @param rho Density of the rod [kg/m^3].
     * @param rod_radius Radius of the rod [m].
     * @param youngs_modulus Young's modulus of the rod [N/m^2].
     * @param poisson_ratio Poisson's ratio of the rod.
     * @param mu Friction coefficient of the rod.
     */
    ElasticRod(int limb_idx, const std::vector<Vec3>& nodes, double rho, double rod_radius,
               double youngs_modulus, double poisson_ratio, double mu);

    /**
     * @brief Default deconstructor.
     */
    ~ElasticRod();

    /**
     * @brief Updates all necessary discrete values and frames for edges
     * in preparation of next timestep.
     *
     * @details Updates discrete tangents, reference frames, material frames,
     * reference twists, edge lengths, and curvatures.
     */
    void prepareForIteration();

    /**
     * @brief Performs a Newton update iteration.
     *
     * @param dx The update std::vector provided by the time stepper and solver.
     * @param offset A constant offset for the rod's DOF index in dx with
     * respect to the global system.
     * @param alpha A Newton update coefficient.
     *
     * @return The max non-theta dx update.
     */
    double updateNewtonX(double* dx, int offset, double alpha = 1.0);

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
     * @param dt The timestep.
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
     * @brief Get the a (3,) position std::vector of the kth node from current
     * timestep.
     *
     * @param k Index of the node.
     *
     * @return The (3,) position std::vector.
     */
    Vec3 getVertex(int k);

    /**
     * @brief Get the (nv, 3) position matrix of all vertices.
     *
     * @return The (nv, 3) position matrix of all vertices.
     */
    MatXN<3> getVertices();

    /**
     * @brief Get the a (3,) position std::vector of the kth node from previous timestep.
     *
     * @param k Index of the node.
     *
     * @return The (3,) position std::vector.
     */
    Vec3 getPreVertex(int k);

    /**
     * @brief Get the a (3,) velocity std::vector of the kth node from current
     * timestep.
     *
     * @param k Index of the node.
     *
     * @return The (3,) velocity std::vector.
     */
    Vec3 getVelocity(int k);

    /**
     * @brief Get the (nv, 3) velocity matrix of all vertices.
     *
     * @return The (nv, 3) velocity matrix of all vertices.
     */
    MatXN<3> getVelocities();

    /**
     * @brief Get the theta of the kth edge from current timestep.
     *
     * @return The theta of the kth edge.
     */
    double getTheta(int k);

    /**
     * @brief Get all the thetas from current timestep.
     *
     * @return The (ne, 1) std::vector of all the thetas.
     */
    VecX getThetas();

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
     * @brief DOF std::vector from the previous timestep.
     *
     * @details Vector of size (ndof, 1).
     */
    VecX x0;

    /**
     * @brief DOF std::vector during and after the current timestep.
     *
     * @details Vector of size (ndof, 1).
     */
    VecX x;

    /**
     * @brief DOF std::vector used to save state during line search.
     *
     * @details Vector of size (ndof, 1).
     */
    VecX x_ls;

    /**
     * @brief Velocity std::vector from the previous timestep.
     *
     * @details Vector of size (ndof, 1).
     */
    VecX u0;

    /**
     * @brief Velocity std::vector during and after the current timestep.
     *
     * @details Vector of size (ndof, 1).
     */
    VecX u;

    /**
     * @brief Vector of current edge lengths [m].
     *
     * @see Computed in computeEdgeLen().
     */
    VecX edge_len;

    /**
     * @brief Reference lengths of the discrete edges [m].
     *
     * @details Computed only once during initialization.
     * Starting configuration is assumed to be resting configuration.
     *
     * @see Computed in setReferenceLength().
     */
    VecX ref_len;

    /**
     * @brief Voronoi lengths of each node [m].
     *
     * @details Computed only once during initialization.
     * Computed from reference lengths.
     *
     * @see Computed in setReferenceLength().
     */
    VecX voronoi_len;

    /**
     * @brief Curvature binormals of each discrete edge.
     *
     * @details Computed during every iteration / timestep to compute
     * the discrete curvatures necessary to compute bending energy.
     *
     * @see Computed in computeKappa().
     */
    MatX kb;

    /**
     * @brief d1 axes of the reference frame during current iteration /
     * timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTimeParallel().
     * @see Used in getRefTwist().
     */
    MatXN<3> d1;

    /**
     * @brief d2 axes of the reference frame during current iteration /
     * timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTimeParallel().
     * @see Used in getRefTwist().
     */
    MatXN<3> d2;

    /**
     * @brief d1 axes of the reference frame from last timestep.
     *
     * @details Matrix of size (ne, 3). Updated by time stepper classes when
     * completing a timestep.
     *
     * @see Used in computeTimeParallel().
     */
    MatXN<3> d1_old;

    /**
     * @brief d2 axes of the reference frame from last timestep.
     *
     * @details Matrix of size (ne, 3). Updated by time stepper classes when
     * completing a timestep.
     */
    MatXN<3> d2_old;

    /**
     * @brief m1 axes of the material frame during current iteration / timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeMaterialDirector().
     * @see Used in computeKappa() to compute discrete curvatures.
     */
    MatXN<3> m1;

    /**
     * @brief m2 axes of the material frame during current iteration / timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeMaterialDirector().
     * @see Used in computeKappa() to compute discrete curvatures.
     */
    MatXN<3> m2;

    /**
     * @brief Edge tangents during the current iteration / timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Updated in computeTangent().
     * @see Used in computeTimeParallel().
     */
    MatXN<3> tangent;

    /**
     * @brief Edge tangents from the previous timestep.
     *
     * @details Matrix of size (ne, 3).
     *
     * @see Used in computeTimeParallel().
     */
    MatXN<3> tangent_old;

    /**
     * @brief Reference twists during the current iteration / timestep.
     *
     * @details Vector of size (ne, 1).
     *
     * @see Updated in getRefTwist().
     * @see Used in computeTwistBar().
     */
    VecX ref_twist;

    /**
     * @brief Reference twists from the previous timestep.
     *
     * @details Vector of size (ne, 1).
     */
    VecX ref_twist_old;

    /**
     * @brief The twists in the resting configuration.
     *
     * @details Vector of size (ne, 1).
     *
     * @see Updated in computeTwistBar().
     */
    VecX twist_bar;

    /**
     * @brief Lumped mass array.
     *
     * @details Vector of size (ndof, 1). Stores the lumped masses of each DOF.
     *
     * @see Updated in setMass().
     *
     */
    VecX mass_array;

    /**
     * @brief Discrete curvatures of the inner nodes.
     *
     * @details Matrix of size (ne, 2). Note that the first and outer values
     * are not used since curvatures only apply to nodes with two edges
     * attached.
     *
     * @see Updated in computeKappa().
     */
    MatXN<2> kappa;

    /**
     * @brief Discrete curvatures of the inner nodes in the resting
     * configuration.
     *
     * @details Matrix of size (ne, 2). Note that the first and outer values
     * are not used since curvatures only apply to nodes with two edges
     * attached. Is assumed to the curvatures computed from the initial starting
     * configuration.
     */
    MatXN<2> kappa_bar;

    /**
     * @brief Array storing whether or not a certain DOF is constrained.
     *
     * @details Size of ndof. Value of 1 if constrained and 0 if unconstrained.
     */
    int* isConstrained;

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
     * to their respective index in x std::vector. The index of the mapping is
     * stored.
     *
     * @see Gets updated in updateMap() and in setup().
     * @see The inverse mapping is fullToUnconsMap.
     */
    int* unconstrainedMap;

    /**
     * @brief Array that maps full to unconstrained indices.
     *
     * @details Size of ndof. Maps all DOF from x std::vector (by array location) to
     * their respective in unconstrained std::vector. The index of the mapping is
     * stored. If a certain DOF is constrained, then the value at that array
     * location is undefined.
     *
     * @see Gets updated in updateMap() and in setup().
     * @see The inverse mapping is unconstrainedMap.
     */
    int* fullToUnconsMap;

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

    // TODO: perhaps split this up into a createJoint() and attachToJoint()
    // functions.
    /**
     * @brief Converts a node on the rod to a joint node.
     *
     * @details This function should NOT be called by the user itself.
     * The ElasticJoint class will handle calling this function.
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
    int* isDOFJoint;

    /**
     * @brief Array indicating whether or not a certain node is a joint node.
     *
     * @details Size of nv. Follows the same rules as isDOFJoint.
     *
     * @see Updated in addJoint().
     */
    int* isNodeJoint;

    /**
     * @brief Array indicating whether or not a certain edge is adjacent
     * to a joint node.
     *
     * @details Size of ne. Follows the same rules as isDOFJoint.
     *
     * @see Updated in addJoint().
     */
    int* isEdgeJoint;

    /**
     * @brief Vector of joint node and limb ids.
     *
     * @details Stores a list of pairs of joint node and joint limb ids.
     * Used to reroute forces and Jacobians to their proper locations in
     * the global force std::vector / Jacobian matrix for nodes that were attached
     * to preexisting joint nodes.
     * Currently of size nv.
     *
     * @see Updated in addJoint().
     */
    std::vector<std::pair<int, int>> joint_ids;

    void setVertexBoundaryCondition(Vec3 position, int k);
    void setThetaBoundaryCondition(double desired_theta, int k);
    void freeVertexBoundaryCondition(int k);
    Vec3 getTangent(int k);

  private:
    void setupMap();

    // TODO: perhaps move these to util.h later as most can be defined as
    // staticmethods?
    void setup(const std::vector<Vec3>& nodes);
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
    static void parallelTransport(const Vec3& d1_1, const Vec3& t1, const Vec3& t2, Vec3& d1_2);
    static void rotateAxisAngle(Vec3& v, const Vec3& z, const double& theta);
    static double signedAngle(const Vec3& u, const Vec3& v, const Vec3& n);
};

#endif  // ELASTIC_ROD_H
