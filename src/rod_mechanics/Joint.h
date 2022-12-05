#ifndef JOINT_H
#define JOINT_H

#include "../eigenIncludes.h"
#include "elasticRod.h"

class Joint
{
public:
    Joint(int node, int limb_idx, vector<shared_ptr<elasticRod>> &limbs);
    int joint_node;
    int joint_limb;

    int ne;

    void updateJoint();
    void addToJoint(int node_num, int limb_idx);

    void updateConnectedNodes(int node_num, int limb_idx);

    vector<shared_ptr<elasticRod>>& limbs;
    Vector3d x;
    vector<pair<int, int>> connected_nodes;  // node_number and limb_idx

    void setup();

    double mass;
    void setMass();

private:
};


#endif
