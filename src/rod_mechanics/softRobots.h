#ifndef SOFTROBOTS_H
#define SOFTROBOTS_H


#include "eigenIncludes.h"
#include "elasticRod.h"
#include "elasticJoint.h"
#include "controllers/rodOpenLoopFileKappabarSetter.h"


class softRobots
{
public:
    softRobots();
    ~softRobots();

    void addLimb(const Vector3d& start, const Vector3d& end, int num_nodes,
                 double rho, double rod_radius, double youngs_modulus, double poisson_ratio, double mu=0.0);
    void addLimb(const vector<Vector3d>& nodes, double rho, double rod_radius,
                 double youngs_modulus, double poisson_ratio, double mu=0.0);

    void createJoint(int limb_idx, int node_idx);
    void addToJoint(int joint_idx, int limb_idx, int node_idx);

    void lockEdge(int limb_idx, int edge_idx);
    void applyInitialVelocities(int limb_idx, const vector<Vector3d>& velocities);

    void setup();

    void addController(const shared_ptr<rodController>& controller);

    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    vector<shared_ptr<rodController>> controllers;

private:
    int num_limbs = 0;

};




#endif
