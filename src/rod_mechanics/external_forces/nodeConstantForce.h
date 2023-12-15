#ifndef NODE_CONSTANT_FORCE_H
#define NODE_CONSTANT_FORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

// declares a structure to hold a force on a specific node of a specific limb
struct limbNodeForceItem{            
    int limb_idx;         
    int node_idx;
    Vector3d force;
};  

class nodeConstantForce : public baseForce
{
public:
    nodeConstantForce(const shared_ptr<softRobots>& m_soft_robots);
    ~nodeConstantForce() override;

    void add_force_to_limb_node(int limb_idx, int node_idx, const Vector3d& force);

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    vector<limbNodeForceItem> limb_node_forces;
};

#endif
