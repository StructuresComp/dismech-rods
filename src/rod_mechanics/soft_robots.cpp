#include "soft_robots.h"
#include "controllers/base_controller.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"

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

void SoftRobots::applyPositionBC(const MatXN<5>& positions) {
    // pos: the first col is limb_idx, second col is node_idx, third col
    // is dx, fourth col is dy, fifth col is dz
    for (int i = 0; i < positions.rows(); i++) {
        int limb_idx = (int)positions(i, 0);
        int node_idx = (int)positions(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->nv) {
            throw std::runtime_error("Invalid limb_idx or node_idx given!");
        }

        Vec3 new_pos = positions.row(i).segment(2, 3);
        limbs[limb_idx]->x.segment(4 * node_idx, 3) = new_pos;
    }
}

void SoftRobots::applyDeltaPositionBC(const MatXN<5>& delta_positions) {
    // delta_pos: the first col is limb_idx, second col is node_idx, third col
    // is dx, fourth col is dy, fifth col is dz
    for (int i = 0; i < delta_positions.rows(); i++) {
        int limb_idx = (int)delta_positions(i, 0);
        int node_idx = (int)delta_positions(i, 1);
        if (limb_idx >= limbs.size() || node_idx >= limbs[limb_idx]->nv) {
            throw std::runtime_error("Invalid limb_idx or node_idx given!");
        }

        Vec3 dpos = delta_positions.row(i).segment(2, 3);
        limbs[limb_idx]->x.segment(4 * node_idx, 3) += dpos;
    }
}

void SoftRobots::applyThetaBC(const MatXN<3>& thetas) {
    // Thetas: the first col is limb_idx, second col is node_idx, third col is
    // theta
    for (int i = 0; i < thetas.rows(); i++) {
        int limb_idx = (int)thetas(i, 0);
        int edge_idx = (int)thetas(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double new_theta = thetas(i, 2);
        limbs[limb_idx]->x(4 * edge_idx + 3) = new_theta;
    }
}

void SoftRobots::applyDeltaThetaBC(const MatXN<3>& delta_thetas) {
    // delta_thetas: the first col is limb_idx, second col is node_idx, third col is
    // dtheta
    for (int i = 0; i < delta_thetas.rows(); i++) {
        int limb_idx = (int)delta_thetas(i, 0);
        int edge_idx = (int)delta_thetas(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double dtheta = delta_thetas(i, 2);
        limbs[limb_idx]->x(4 * edge_idx + 3) += dtheta;
    }
}

void SoftRobots::applyCurvatureBC(const MatXN<4>& curvatures) {
    // curvatures: the first col is limb_idx, second col is edge_idx,
    // third col is cx, fourth col is cy
    for (int i = 0; i < curvatures.rows(); i++) {
        int limb_idx = (int)curvatures(i, 0);
        int edge_idx = (int)curvatures(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne || edge_idx < 1) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double new_cx = curvatures(i, 2);
        double new_cy = curvatures(i, 3);

        limbs[limb_idx]->kappa_bar(edge_idx, 0) = new_cx;
        limbs[limb_idx]->kappa_bar(edge_idx, 1) = new_cy;
    }
}

void SoftRobots::applyDeltaCurvatureBC(const MatXN<4>& delta_curvatures) {
    // delta_curvatures: the first col is limb_idx, second col is edge_idx,
    // third col is delta cx, fourth col is delta cy
    for (int i = 0; i < delta_curvatures.rows(); i++) {
        int limb_idx = (int)delta_curvatures(i, 0);
        int edge_idx = (int)delta_curvatures(i, 1);
        if (limb_idx >= limbs.size() || edge_idx >= limbs[limb_idx]->ne || edge_idx < 1) {
            throw std::runtime_error("Invalid limb_idx or edge_idx given!");
        }

        double dcx = delta_curvatures(i, 2);
        double dcy = delta_curvatures(i, 3);

        limbs[limb_idx]->kappa_bar(edge_idx, 0) += dcx;
        limbs[limb_idx]->kappa_bar(edge_idx, 1) += dcy;
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
