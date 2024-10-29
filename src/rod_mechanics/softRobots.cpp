#include "softRobots.h"


softRobots::softRobots() = default;

softRobots::~softRobots() = default;


void softRobots::addLimb(const Vector3d& start, const Vector3d& end, int num_nodes,
                        double rho, double rod_radius, double youngs_modulus, double poisson_ratio, double mu) {
    limbs.push_back(make_shared<elasticRod>(num_limbs, start, end, num_nodes, rho,
                                            rod_radius, youngs_modulus, poisson_ratio, mu));
    num_limbs++;
}


void softRobots::addLimb(const vector<Eigen::Vector3d> &nodes, double rho, double rod_radius, double youngs_modulus,
                        double poisson_ratio, double mu) {
    limbs.push_back(make_shared<elasticRod>(num_limbs, nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu));
    num_limbs++;
}


void softRobots::createJoint(int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv-1;
    }
    joints.push_back(make_shared<elasticJoint>(node_idx, limb_idx, limbs));
}


void softRobots::addToJoint(int joint_idx, int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv-1;
    }
    joints[joint_idx]->addToJoint(node_idx, limb_idx);
}


void softRobots::lockEdge(int limb_idx, int edge_idx) {
    shared_ptr<elasticRod> limb = limbs[limb_idx];
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx), edge_idx);
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx+1), edge_idx+1);
    limb->setThetaBoundaryCondition(0.0, edge_idx);
}

void softRobots::applyInitialVelocities(int limb_idx, const vector<Vector3d> &velocities) {
    shared_ptr<elasticRod> limb = limbs[limb_idx];
    if (limb->nv != velocities.size()) {
        throw runtime_error("The number of nodes (" + to_string(limb->nv) +
                            ") and velocities (" + to_string(velocities.size()) + ") given did not match!");
    }
    for (int i = 0; i < limb->nv; i++) {
        limb->u.segment(4*i, 3) = limb->u0.segment(4*i, 3) = velocities.at(i);
    }
}

void softRobots::applyPositionBC(const MatrixXd &positions) {
    // Positions: the first col is limb_idx, second col is node_idx, third col is x, fourth col is y, fifth col is z 
    if (positions.cols() != 5) {
        throw runtime_error("The positions matrix should have 5 columns!");
    }

    for (int i = 0; i < positions.rows(); i++) {
        int limb_idx = positions(i, 0);
        int node_idx = positions(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->nv) {
            throw runtime_error("Invalid limb_idx or node_idx given!");
        }
        
        Vector3d pos = positions.row(i).segment(2, 3).transpose();
        limbs[limb_idx]->x.segment(4*node_idx, 3) += pos;
    }
}

void softRobots::applyTwistBC(const MatrixXd &twists) {
    // Twists: the first col is limb_idx, second col is node_idx, third col is theta 
    if (twists.cols() != 3) {
        throw runtime_error("The twist input matrix should have 3 columns!");
    }

    for (int i = 0; i < twists.rows(); i++) {
        // the first col is limb_idx, second col is node_idx, third col is x, fourth col is y, fifth col is z 
        int limb_idx = twists(i, 0);
        int edge_idx = twists(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne) {
            throw runtime_error("Invalid limb_idx or edge_idx given!");
        }
        
        double theta = twists(i, 2);
        limbs[limb_idx]->x(4*edge_idx + 3) += theta;
    }
}

void softRobots::applyCurvatureBC(const MatrixXd &Curvatures) {
    // Curvatures: the first col is limb_idx, second col is node_idx, third col is cx, fourth col is cy, fifth col is cz 
    if (Curvatures.cols() != 4) {
        throw runtime_error("The twist input matrix should have 3 columns!");
    }

    for (int i = 0; i < Curvatures.rows(); i++) {
        // the first col is limb_idx, second col is node_idx, third col is x, fourth col is y, fifth col is z 
        int limb_idx = Curvatures(i, 0);
        int node_idx = Curvatures(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->ne || node_idx < 1) {
            throw runtime_error("Invalid limb_idx or edge_idx given!");
        }

        // get material frames   
        // Vector3d curvature = Curvatures.row(i).segment(2, 3).transpose();
        double cx = Curvatures(i, 2);
        double cy = Curvatures(i, 3);

        // Vector3d m1e = limbs[limb_idx]->m1.row(node_idx - 1);
        // Vector3d m2e = limbs[limb_idx]->m2.row(node_idx - 1);
        // Vector3d m1f = limbs[limb_idx]->m1.row(node_idx);
        // Vector3d m2f = limbs[limb_idx]->m2.row(node_idx);
        
        limbs[limb_idx]->kappa_bar(i, 0) += cx;
        limbs[limb_idx]->kappa_bar(i, 1) += cy;
    }
}

void softRobots::setup() {
    for (const auto& joint : joints)  {
        joint->setup();
    }
}


void softRobots::addController(const shared_ptr<baseController>& controller) {
    controllers.emplace_back(controller);
}
