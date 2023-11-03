#ifndef ELASTICROD_H
#define ELASTICROD_H

#include "eigenIncludes.h"

class elasticRod
{
    // NOTE: probably could move more stuff to private
    public:
    elasticRod(int limb_idx, const Vector3d& start, const Vector3d& end, int num_nodes,
               double rho, double rod_radius, double youngs_modulus, double poisson_ratio, double mu);
    elasticRod(int limb_idx, const vector<Vector3d>& nodes, double rho, double rod_radius,
               double youngs_modulus, double poisson_ratio, double mu);
    ~elasticRod();

    void setVertexBoundaryCondition(Vector3d position, int k);
    void setThetaBoundaryCondition(double desired_theta, int k);
    void prepareForIteration();
    double updateNewtonX(double *dx, int offset, double alpha=1.0);
    void updateGuess(double weight, double dt);
    void enable2DSim() const;

    int limb_idx;

    // utility functions
    Vector3d getVertex(int k);
    Vector3d getPreVertex(int k);
    Vector3d getVelocity(int k);
    Vector3d getTangent(int k);
    double getTheta(int k);
    void updatePhis(double phi1, double phi2);

    // Elastic stiffness values
    double poisson_ratio;
    double youngM, shearM; // Young's and shear modulus
    double EA;             // stretching stiffness
    double EI;             // bending stiffness
    double GJ;             // twisting stiffness

    int nv;                    // number of vertices
    int ne;                    // number of edges = nv-1
    int ndof;                  // number of degrees of freedom = 3*nv + ne
    int ncons;                 // number of constrained dof
    int uncons;                // number of unconstrained dof
    double rho;                // density
    double rod_radius;           // cross-sectional radius of the rod
    double cross_sectional_area; // cross-sectional area of the rod
    double dm; // mass per segment
    double mu; // friction coefficient
    
    // Total length
    double rod_length;
    // Bending curvature angle phi (from the end to the tip of the limb, for each edge: phi_e = phi/ne)
    double phi1;
    double phi2;
    bool actuated;

    // PI
    const double PI =  3.14159265358979323846264;
    // Edge length
    VectorXd edge_len;
    // curvature binormal
    MatrixXd kb;
    // reference lengths
    VectorXd ref_len;
    // Voronoi lengths
    VectorXd voronoi_len;

    vector<Vector3d> all_nodes;

    // dof vector before time step
    VectorXd x0;
    // dof vector after time step
    VectorXd x;
    // dof vector for line search
    VectorXd x_ls;
    // velocity vector
    VectorXd u;
    VectorXd u0;

    // Reference directors
    MatrixXd d1;
    MatrixXd d2;
    MatrixXd d1_old;
    MatrixXd d2_old;
    // Material directors
    MatrixXd m1;
    MatrixXd m2;
    // Tangents
    MatrixXd tangent;
    MatrixXd tangent_old;
    // Reference twist
    VectorXd ref_twist;
    VectorXd ref_twist_old;
    // Undeformed twist
    VectorXd twist_bar;
    // lumped mass
    VectorXd mass_array;
    // lumped mass at unconstrained dofs
    VectorXd mUncons;
    // Curvature
    MatrixXd kappa;
    // Natural curvature
    MatrixXd kappa_bar;
    // twist angle theta;
    VectorXd theta;

    // boundary conditions
    int *isConstrained;
    int getIfConstrained(int k) const;
    int *unconstrainedMap;
    int *fullToUnconsMap;

    void updateMap();

    void freeVertexBoundaryCondition(int k);

    void addJoint(int node_num, bool remove_dof, int joint_node, int joint_limb);
    int unique_dof;
    int *isDOFJoint;
    int *isNodeJoint;
    int *isEdgeJoint;
    int *DOFoffsets;
    vector<pair<int, int>> joint_ids;

private:
    void setupMap();

    // NOTE: perhaps move these to util.h later?
    void setup();
    void computeElasticStiffness();
    void setMass();
    void setReferenceLength();
    void computeTimeParallel();
    void computeTangent();
    void computeSpaceParallel();
    void computeMaterialDirector();
    void computeKappa();
    void computeAngle2KappaBar();
    void computeTwistBar();
    void computeEdgeLen();
    void getRefTwist();
    static void parallelTransport(const Vector3d &d1_1,const Vector3d &t1,const Vector3d &t2,Vector3d &d1_2);
    static void rotateAxisAngle(Vector3d &v, const Vector3d &z, const double &theta);
    static double signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n);
};

#endif
