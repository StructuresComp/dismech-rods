#include "soft_robots.h"

SoftRobots::SoftRobots() = default;

SoftRobots::~SoftRobots() = default;

void SoftRobots::addLimb(const Vec3& start, const Vec3& end, int num_nodes, double rho,
                         double rod_radius, double youngs_modulus, double poisson_ratio,
                         double mu) {
    limbs.push_back(std::make_shared<ElasticRod>(num_limbs, start, end, num_nodes, rho, rod_radius,
                                                 youngs_modulus, poisson_ratio, mu));
    num_limbs++;
}

void SoftRobots::addLimb(const std::vector<Vec3>& nodes, double rho, double rod_radius,
                         double youngs_modulus, double poisson_ratio, double mu) {
    limbs.push_back(std::make_shared<ElasticRod>(num_limbs, nodes, rho, rod_radius, youngs_modulus,
                                                 poisson_ratio, mu));
    num_limbs++;
}

void SoftRobots::createJoint(int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv - 1;
    }
    joints.push_back(std::make_shared<ElasticJoint>(node_idx, limb_idx, limbs));
}

void SoftRobots::addToJoint(int joint_idx, int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv - 1;
    }
    joints[joint_idx]->addToJoint(node_idx, limb_idx);
}

void SoftRobots::lockEdge(int limb_idx, int edge_idx) {
    std::shared_ptr<ElasticRod> limb = limbs[limb_idx];
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx), edge_idx);
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx + 1), edge_idx + 1);
    limb->setThetaBoundaryCondition(0.0, edge_idx);
}

void SoftRobots::applyInitialVelocities(int limb_idx, const std::vector<Vec3>& velocities) {
    std::shared_ptr<ElasticRod> limb = limbs[limb_idx];
    if (limb->nv != velocities.size()) {
        throw std::runtime_error("The number of nodes (" + std::to_string(limb->nv) +
                                 ") and velocities (" + std::to_string(velocities.size()) +
                                 ") given did not match!");
    }
    for (int i = 0; i < limb->nv; i++) {
        limb->u.segment(4 * i, 3) = limb->u0.segment(4 * i, 3) = velocities.at(i);
    }
}

void SoftRobots::applyPositionBC(const MatXN<5>& delta_pos) {
    // delta_pos: the first col is limb_idx, second col is node_idx, third col
    // is dx, fourth col is dy, fifth col is dz
    for (int i = 0; i < delta_pos.rows(); i++) {
        int limb_idx = delta_pos(i, 0);
        int node_idx = delta_pos(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->nv) {
            throw std::runtime_error("Invalid limb_idx or node_idx given!");
        }

        Vec3 dpos = delta_pos.row(i).segment(2, 3).transpose();
        limbs[limb_idx]->x.segment(4 * node_idx, 3) += dpos;
    }
}

void SoftRobots::applyTwistBC(const MatXN<3>& delta_twist) {
    // Twists: the first col is limb_idx, second col is node_idx, third col is
    // dtheta
    for (int i = 0; i < delta_twist.rows(); i++) {
        int limb_idx = delta_twist(i, 0);
        int edge_idx = delta_twist(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double dtheta = delta_twist(i, 2);
        limbs[limb_idx]->x(4 * edge_idx + 3) += dtheta;
    }
}

void SoftRobots::applyCurvatureBC(const MatXN<4>& delta_curvatures) {
    // delta_curvatures: the first col is limb_idx, second col is node_idx,
    // third col is cx, fourth col is cy
    for (int i = 0; i < delta_curvatures.rows(); i++) {
        int limb_idx = delta_curvatures(i, 0);
        int node_idx = delta_curvatures(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->ne || node_idx < 1) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double cx = delta_curvatures(i, 2);
        double cy = delta_curvatures(i, 3);

        limbs[limb_idx]->kappa_bar(i, 0) += cx;
        limbs[limb_idx]->kappa_bar(i, 1) += cy;
    }
}

void SoftRobots::setup() {
    for (const auto& joint : joints) {
        joint->setup();
    }
}

void SoftRobots::addController(const std::shared_ptr<BaseController>& controller) {
    controllers.emplace_back(controller);
}
