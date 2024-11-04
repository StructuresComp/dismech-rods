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

void softRobots::applyPositionBC(const Matrix<double, Dynamic, 5>  &delta_pos) {
    // delta_pos: the first col is limb_idx, second col is node_idx, third col is dx, fourth col is dy, fifth col is dz
    for (int i = 0; i < delta_pos.rows(); i++) {
        int limb_idx = delta_pos(i, 0);
        int node_idx = delta_pos(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->nv) {
            throw runtime_error("Invalid limb_idx or node_idx given!");
        }

        Vector3d dpos = delta_pos.row(i).segment(2, 3).transpose();
        limbs[limb_idx]->x.segment(4*node_idx, 3) += dpos;
    }
}

void softRobots::applyTwistBC(const Matrix<double, Dynamic, 3> &delta_twist) {
    // Twists: the first col is limb_idx, second col is node_idx, third col is dtheta
    for (int i = 0; i < delta_twist.rows(); i++) {
        int limb_idx = delta_twist(i, 0);
        int edge_idx = delta_twist(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne) {
            throw runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double dtheta = delta_twist(i, 2);
        limbs[limb_idx]->x(4*edge_idx + 3) += dtheta;
    }
}

void softRobots::applyCurvatureBC(const Matrix<double, Dynamic, 4> &delta_curvatures) {
    // delta_curvatures: the first col is limb_idx, second col is node_idx, third col is cx, fourth col is cy
    for (int i = 0; i < delta_curvatures.rows(); i++) {
        int limb_idx = delta_curvatures(i, 0);
        int node_idx = delta_curvatures(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->ne || node_idx < 1) {
            throw runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double cx = delta_curvatures(i, 2);
        double cy = delta_curvatures(i, 3);

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
