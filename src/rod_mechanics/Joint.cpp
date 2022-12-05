#include "Joint.h"

Joint::Joint(int node, int limb_idx, vector<shared_ptr<elasticRod>> &m_limbs)
            : joint_node(node), joint_limb(limb_idx), limbs(m_limbs)
{
    ne = 0;
    updateConnectedNodes(joint_node, joint_limb);
}

void Joint::updateConnectedNodes(int node_num, int limb_idx) {
    int nv = limbs[limb_idx]->nv;
    if (node_num == 0) {
        pair<int, int> node_and_limb(1, joint_limb);
        connected_nodes.push_back(node_and_limb);
        ne += 1;
    }
    else if (node_num == nv-1) {
        pair<int, int> node_and_limb(nv-2, joint_limb);
        connected_nodes.push_back(node_and_limb);
        ne += 1;
    }
    else {
        pair<int, int> node_and_limb1(node_num-1, joint_limb);
        connected_nodes.push_back(node_and_limb1);
        pair<int, int> node_and_limb2(node_num+1, joint_limb);
        connected_nodes.push_back(node_and_limb2);
        ne += 2;
    }
}


void Joint::updateJoint() {
    x(0) = limbs[joint_limb]->x(4*joint_node);
    x(1) = limbs[joint_limb]->x(4*joint_node+1);
    x(2) = limbs[joint_limb]->x(4*joint_node+2);
}


void Joint::addToJoint(int node_num, int limb_idx) {
    limbs[limb_idx]->joint_nodes.push_back(node_num);
    limbs[limb_idx]->addJoint(this);
    updateConnectedNodes(node_num, limb_idx);
}


void Joint::setup() {
}


void Joint::setMass() {
    mass = 0;
    double curr_mass;
    shared_ptr<elasticRod> curr_limb;
    for (const auto& connections : connected_nodes ) {
        int node_num = connections[0];
        int limb_num = connections[1];
        curr_limb = limbs[limb_num];

        curr_mass = curr_limb->rodLength * curr_limb->crossSectionalArea * curr_limb->rho / curr_limb->ne;
        if (node_num == 0 || node_num == curr_limb->nv-1) curr_mass = curr_mass / 2.0;
        mass += curr_mass;
    }
}
