#ifndef NON_UNIFORM_CONSTANT_FORCE_H
#define NON_UNIFORM_CONSTANT_FORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

// declares a structure to hold a force on a specific node of a specific limb
struct limbNodeForceItem{            
    int limb_idx;         
    int node_idx;
    Vector3d force;
};  

class nonUniformConstantForce : public baseForce
{
public:
    nonUniformConstantForce(const shared_ptr<softRobots>& m_soft_robots);
    ~nonUniformConstantForce() override;

    void add_force_to_limb_node(int limb_idx, int node_idx, const Vector3d& force);

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    vector<limbNodeForceItem> limb_node_forces;
};

#endif
