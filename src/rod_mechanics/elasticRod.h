#ifndef ELASTICROD_H
#define ELASTICROD_H

#include <map>
#include <array>
#include <cassert>
#include "../eigenIncludes.h"

class elasticRod
{
    public:
    elasticRod(Vector3d start, Vector3d end, int num_nodes,
               double m_rho, double m_rodRadius, double m_dt,
                double m_youngM, double m_shearM);
    elasticRod(MatrixXd initialNodes, MatrixXd undeformed,
    double m_rho, double m_rodRadius, double m_dt,
    double m_youngM, double m_shearM, double m_rodLength, VectorXd m_theta);
    ~elasticRod();
    void setup();
    void setMass();
    void setReferenceLength();
    void setVertexBoundaryCondition(Vector3d position, int k);
    void setThetaBoundaryCondition(double desiredTheta, int k);
    void updateTimeStep();
    void computeElasticStiffness();
    void prepareForIteration();
    void updateNewtonX(double *dx, int offset, double alpha=1.0);
    void updateGuess(double weight);

//    void addInitRod(Vector3d start, Vector3d end, int num_nodes);
//    void addRod(int limb_num, int connection_node, Vector3d end, int num_nodes);
//    void addJoint(int node1, int limb1, int node2, int limb2);
//
//    struct Limb {
//        int start;
//        int end;
//        int num_nodes;
//        double limb_length;
//    };
//    vector<Limb> limbs;

    // utility functions
    Vector3d getVertex(int k);
    Vector3d getPreVertex(int k);
    Vector3d getVelocity(int k);
    Vector3d getTangent(int k);
    double getTheta(int k);
    void printInfo();

    // Should be taken out of this class
    void computeTimeParallel();
    void computeTangent(const VectorXd &x, MatrixXd &tangentLocal);
    void parallelTansport(const Vector3d &d1_1,const Vector3d &t1,const Vector3d &t2,Vector3d &d1_2);
    void computeSpaceParallel();
    void computeMaterialDirector();
    void computeKappa();
    void computeEdgeLen();
    void getRefTwist();
    void rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta);
    double signedAngle(const Vector3d &u,const Vector3d &v,const Vector3d &n);


    // Elastic stiffness values
    double youngM, shearM; // Young's and shear modulus
    double EA; // stretching stiffness
    double EI; // bending stiffness
    double GJ; // twisting stiffness

    int nv; // number of vertices
    int ne; // number of edges = nv-1
    int ndof; // number of degrees of freedom = 3*nv + ne
    int ncons; // number of constrained dof
    int uncons; // number of unconstrained dof
    double rho; // density
    double rodRadius; // cross-sectional radius of the rod
    double crossSectionalArea; // cross-sectional area of the rod
    double dt; // time step
    double dm; // mass per segment

    // Total length
    double rodLength;
    // Edge length
    VectorXd edgeLen;
    // curvature binormal
    MatrixXd kb;
    // reference lengths
    VectorXd refLen;
    // Voronoi lengths
    VectorXd voronoiLen;

    vector<Vector3d> all_nodes;
    vector<array<int,2>> stretching_nodes;
    vector<array<int,3>> bending_nodes;
    vector<array<int,2>> twisting_nodes;

    int num_stretching;
    int num_bending;
    int num_twisting;

    map<int, vector<int>> node_node_neighbors;
    map<int, vector<int>> node_edge_neighbors;
    map<int, array<int, 2>> edge_node_map;
    map<int, int> edge_limb_map;

    // dof vector before time step
    VectorXd x0;
    // dof vector after time step
    VectorXd x;
    // velocity vector
    VectorXd u;

    // nodes
    MatrixXd nodes, nodesUndeformed;
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
    VectorXd refTwist;
    VectorXd refTwist_old;
    // Undeformed twist
    VectorXd undeformedTwist;
    // lumped mass
    VectorXd massArray;
    // lumped mass at unconstrained dofs
    VectorXd mUncons;
    // Curvature
    MatrixXd kappa;
    // Natural curvature
    MatrixXd kappaBar;
    // twist angle theta;
    VectorXd theta;

    // boundary conditions
    int* isConstrained;
    int getIfConstrained(int k);
    int* unconstrainedMap;
    int* fullToUnconsMap;
    void setupMap();

    void updateMap();

    void setVelocity(int k);

    void freeVertexBoundaryCondition(int k);

    VectorXd xold;

    void addJoint(int node_num, bool remove_dof);
    int unique_dof;
    int* isDOFJoint;
    int* isEdgeJoint;
    int* DOFoffsets;
    int* unconsToDOFMap;

    private:

//    map<
};

#endif
