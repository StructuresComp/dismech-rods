#include "elastic_joint.h"
#include "elastic_rod.h"

ElasticJoint::ElasticJoint(int node, int limb_idx,
                           const std::vector<std::shared_ptr<ElasticRod>>& limbs)
    : joint_node(node), joint_limb(limb_idx), limbs(limbs) {
    ne = 0;
    limbs[limb_idx]->addJoint(joint_node, false, 0, 0);
    updateConnectedNodes(joint_node, joint_limb, false);
}

void ElasticJoint::setup() {
    num_bending_combos = 0;
    int n1, n2;
    int sgn1, sgn2;
    int theta1_i, theta2_i;
    for (int i = 0; i < ne; i++) {
        n1 = connected_nodes[i].first;
        bending_twist_signs[i] == 1 ? sgn1 = 1 : sgn1 = -1;
        bending_twist_signs[i] == 1 ? theta1_i = 4 * n1 + 3 : theta1_i = 4 * n1 - 1;
        for (int j = i + 1; j < ne; j++) {
            n2 = connected_nodes[j].first;
            bending_twist_signs[j] == 1 ? sgn2 = -1 : sgn2 = 1;
            bending_twist_signs[j] == 1 ? theta2_i = 4 * n2 + 3 : theta2_i = 4 * n2 - 1;
            sgns.emplace_back(sgn1, sgn2);
            theta_inds.emplace_back(theta1_i, theta2_i);
            num_bending_combos++;
        }
    }

    updateJoint();
    updateRods();
    x0 = x;
    u0 = u;

    // All nodal quantities will have multiple values since there are
    // multiple possible connections to the joint
    ref_len = VecX(ne);
    voronoi_len = VecX(num_bending_combos);

    tangents = MatX::Zero(ne, 3);

    for (int i = 0; i < num_bending_combos; i++) {
        d1.emplace_back(MatX::Zero(2, 3));
        d2.emplace_back(MatX::Zero(2, 3));
        d1_old.emplace_back(MatX::Zero(2, 3));
        d2_old.emplace_back(MatX::Zero(2, 3));
        m1.emplace_back(MatX::Zero(2, 3));
        m2.emplace_back(MatX::Zero(2, 3));
    }
    ref_twist = VecX::Zero(num_bending_combos);
    ref_twist_old = VecX::Zero(num_bending_combos);

    kb = MatX::Zero(num_bending_combos, 3);
    kappa = MatX::Zero(num_bending_combos, 2);
    twist_bar = VecX::Zero(num_bending_combos);
    edge_len = VecX::Zero(ne);

    setReferenceLength();

    setMass();

    computeTangent();

    createReferenceDirectors();

    computeMaterialDirectors();

    computeKappa();

    kappa_bar = kappa;

    getRefTwist();

    computeTwistBar();

    computeEdgeLen();

    d1_old = d1;
    d2_old = d2;
    tangents_old = tangents;
    ref_twist_old = ref_twist;
}

void ElasticJoint::updateConnectedNodes(int node_num, int limb_idx, bool remove_dof) {
    int nv = limbs[limb_idx]->nv;
    if (node_num == 0) {
        std::pair<int, int> node_and_limb(1, limb_idx);
        connected_nodes.push_back(node_and_limb);
        bending_twist_signs.push_back(-1);
        if (remove_dof) {
            std::pair<int, int> node_and_limb2(0, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else if (node_num == nv - 1) {
        std::pair<int, int> node_and_limb(nv - 2, limb_idx);
        connected_nodes.push_back(node_and_limb);
        bending_twist_signs.push_back(1);
        if (remove_dof) {
            std::pair<int, int> node_and_limb2(nv - 1, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else {
        std::pair<int, int> node_and_limb1(node_num - 1, limb_idx);
        connected_nodes.push_back(node_and_limb1);
        bending_twist_signs.push_back(1);
        std::pair<int, int> node_and_limb2(node_num + 1, limb_idx);
        connected_nodes.push_back(node_and_limb2);
        bending_twist_signs.push_back(-1);
        if (remove_dof) {
            std::pair<int, int> node_and_limb3(node_num, limb_idx);
            replaced_nodes.push_back(node_and_limb3);
        }
        ne += 2;
    }
}

void ElasticJoint::updateJoint() {
    x(0) = limbs[joint_limb]->x(4 * joint_node);
    x(1) = limbs[joint_limb]->x(4 * joint_node + 1);
    x(2) = limbs[joint_limb]->x(4 * joint_node + 2);
    x0(0) = limbs[joint_limb]->x0(4 * joint_node);
    x0(1) = limbs[joint_limb]->x0(4 * joint_node + 1);
    x0(2) = limbs[joint_limb]->x0(4 * joint_node + 2);
    u(0) = limbs[joint_limb]->u(4 * joint_node);
    u(1) = limbs[joint_limb]->u(4 * joint_node + 1);
    u(2) = limbs[joint_limb]->u(4 * joint_node + 2);
    u0(0) = limbs[joint_limb]->u0(4 * joint_node);
    u0(1) = limbs[joint_limb]->u0(4 * joint_node + 1);
    u0(2) = limbs[joint_limb]->u0(4 * joint_node + 2);
}

void ElasticJoint::updateRods() {
    std::shared_ptr<ElasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (const auto& node_and_limb : replaced_nodes) {
        num_node = node_and_limb.first;
        limb_idx = node_and_limb.second;
        curr_limb = limbs[limb_idx];

        curr_limb->x(4 * num_node) = x(0);
        curr_limb->x(4 * num_node + 1) = x(1);
        curr_limb->x(4 * num_node + 2) = x(2);
    }
}

void ElasticJoint::addToJoint(int node_num, int limb_idx) {
    limbs[limb_idx]->addJoint(node_num, true, joint_node, joint_limb);
    updateConnectedNodes(node_num, limb_idx, true);
}

void ElasticJoint::setReferenceLength() {
    std::shared_ptr<ElasticRod> curr_limb;
    int node, limb_idx;
    for (int i = 0; i < ne; i++) {
        node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];
        //        ref_len(i) = curr_limb->rodLength / curr_limb->ne;
        //        ref_len(i) = (curr_limb->x.segment(4*(node+1), 3) -
        //        curr_limb->x.segment(4*node, 3)).norm();
        ref_len(i) = (x - curr_limb->x.segment(4 * node, 3)).norm();
    }

    int curr_index = 0;
    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            voronoi_len(curr_index) = 0.5 * (ref_len(i) + ref_len(j));
            curr_index++;
        }
    }
}

void ElasticJoint::computeTangent() {
    // NOTE: all tangents are pointing toward the joint
    std::shared_ptr<ElasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];

        if (bending_twist_signs[i] == 1) {
            tangents.row(i) = x - curr_limb->x.segment(4 * num_node, 3);
            tangents.row(i) /= tangents.row(i).norm();
        }
        else {
            tangents.row(i) = curr_limb->x.segment(4 * num_node, 3) - x;
            tangents.row(i) /= tangents.row(i).norm();
        }
    }
}

void ElasticJoint::createReferenceDirectors() {
    Vec3 t0, t1, tmp1, tmp2;
    int curr_iter = 0;
    int sgn1, sgn2;
    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            t0 = tangents.row(i) * sgn1;
            t1 = tangents.row(j) * sgn2;
            tmp1 << 0, 0, -1;
            tmp2 = t0.cross(tmp1);

            if (fabs(tmp2.norm()) < 1.0e-6) {
                tmp1 << 0, 1, 0;
                tmp2 = t0.cross(tmp1);
            }

            tmp2 /= tmp2.norm();
            d1[curr_iter].row(0) = tmp2;
            d2[curr_iter].row(0) = t0.cross(tmp2);

            parallelTransport(tmp2, t0, t1, tmp1);

            d1[curr_iter].row(1) = tmp1;
            d2[curr_iter].row(1) = t1.cross(tmp1);

            curr_iter++;
        }
    }
}

void ElasticJoint::computeMaterialDirectors() {
    double cs1, ss1;
    double cs2, ss2;
    int theta1_i, theta2_i;
    double angle1;
    double angle2;
    std::shared_ptr<ElasticRod> limb1, limb2;
    int l1, l2;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        l1 = connected_nodes[i].second;
        for (int j = i + 1; j < ne; j++) {
            l2 = connected_nodes[j].second;

            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            theta1_i = theta_inds[curr_iter][0];
            theta2_i = theta_inds[curr_iter][1];

            limb1 = limbs[l1];
            limb2 = limbs[l2];
            angle1 = limb1->x[theta1_i] * sgn1;
            angle2 = limb2->x[theta2_i] * sgn2;
            cs1 = cos(angle1);
            ss1 = sin(angle1);
            cs2 = cos(angle2);
            ss2 = sin(angle2);

            m1[curr_iter].row(0) = cs1 * d1[curr_iter].row(0) + ss1 * d2[curr_iter].row(0);
            m2[curr_iter].row(0) = -ss1 * d1[curr_iter].row(0) + cs1 * d2[curr_iter].row(0);

            m1[curr_iter].row(1) = cs2 * d1[curr_iter].row(1) + ss2 * d2[curr_iter].row(1);
            m2[curr_iter].row(1) = -ss2 * d1[curr_iter].row(1) + cs2 * d2[curr_iter].row(1);

            curr_iter++;
        }
    }
}

void ElasticJoint::computeKappa() {
    Vec3 t0, t1;
    Vec3 m1e, m2e, m1f, m2f;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            t0 = tangents.row(i) * sgn1;
            t1 = tangents.row(j) * sgn2;
            kb.row(curr_iter) = 2.0 * t0.cross(t1) / (1.0 + t0.dot(t1));
            curr_iter++;
        }
    }

    curr_iter = 0;
    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            m1e = m1[curr_iter].row(0);
            m2e = m2[curr_iter].row(0);
            m1f = m1[curr_iter].row(1);
            m2f = m2[curr_iter].row(1);
            kappa(curr_iter, 0) = 0.5 * (kb.row(curr_iter)).dot(m2e + m2f);
            kappa(curr_iter, 1) = -0.5 * (kb.row(curr_iter)).dot(m1e + m1f);
            curr_iter++;
        }
    }
}

void ElasticJoint::parallelTransport(const Vec3& d1_1, const Vec3& t1, const Vec3& t2, Vec3& d1_2) {
    Vec3 b;
    Vec3 n1, n2;

    b = t1.cross(t2);

    if (b.norm() == 0)
        d1_2 = d1_1;
    else {
        b = b / b.norm();
        b = b - b.dot(t1) * t1;
        b = b / b.norm();
        b = b - b.dot(t1) * t2;
        b = b / b.norm();

        n1 = t1.cross(b);
        n2 = t2.cross(b);
        d1_2 = d1_1.dot(t1) * t2 + d1_1.dot(n1) * n2 + d1_1.dot(b) * b;
        d1_2 = d1_2 - d1_2.dot(t2) * t2;
        d1_2 = d1_2 / d1_2.norm();
    }
}

void ElasticJoint::rotateAxisAngle(Vec3& v, const Vec3& z, const double& theta) {
    // Compute the std::vector when it rotates along another std::vector into certain
    // angle
    if (theta != 0)  // if theta=0, v = v
    {
        double cs, ss;
        cs = cos(theta);
        ss = sin(theta);
        v = cs * v + ss * z.cross(v) + z.dot(v) * (1.0 - cs) * z;
    }
}

double ElasticJoint::signedAngle(const Vec3& u, const Vec3& v, const Vec3& n) {
    // Compute the angle between two vectors
    Vec3 w = u.cross(v);
    double angle = atan2(w.norm(), u.dot(v));
    if (n.dot(w) < 0)
        return -angle;
    else
        return angle;
}

void ElasticJoint::getRefTwist() {
    Vec3 u0, u1, t0, t1, ut;
    double sgnAngle;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            u0 = d1[curr_iter].row(0);
            u1 = d1[curr_iter].row(1);
            t0 = tangents.row(i) * sgn1;
            t1 = tangents.row(j) * sgn2;
            parallelTransport(u0, t0, t1, ut);
            rotateAxisAngle(ut, t1, ref_twist_old(curr_iter));

            sgnAngle = signedAngle(ut, u1, t1);
            ref_twist(curr_iter) = ref_twist_old(curr_iter) + sgnAngle;
            curr_iter++;
        }
    }
}

void ElasticJoint::computeTwistBar() {
    double theta_i, theta_f;
    int l1, l2;
    int theta1_i, theta2_i;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        l1 = connected_nodes[i].second;
        for (int j = i + 1; j < ne; j++) {
            l2 = connected_nodes[j].second;

            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            theta1_i = theta_inds[curr_iter][0];
            theta2_i = theta_inds[curr_iter][1];

            theta_i = limbs[l1]->x[theta1_i] * sgn1;
            theta_f = limbs[l2]->x[theta2_i] * sgn2;
            twist_bar(curr_iter) = theta_f - theta_i + ref_twist(curr_iter);
            curr_iter++;
        }
    }
}

void ElasticJoint::computeEdgeLen() {
    std::shared_ptr<ElasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];
        edge_len[i] = (x - curr_limb->x.segment(4 * num_node, 3)).norm();
    }
}

void ElasticJoint::computeTimeParallel() {
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vec3 t0_0, t1_0, t0_1, t1_1, d1_0, d1_1;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            t0_0 = tangents_old.row(i) * sgn1;
            t1_0 = tangents.row(i) * sgn1;
            t0_1 = tangents_old.row(j) * sgn2;
            t1_1 = tangents.row(j) * sgn2;

            parallelTransport(d1_old[curr_iter].row(0), t0_0, t1_0, d1_0);
            parallelTransport(d1_old[curr_iter].row(1), t0_1, t1_1, d1_1);

            d1[curr_iter].row(0) = d1_0;
            d2[curr_iter].row(0) = t1_0.cross(d1_0);
            d1[curr_iter].row(1) = d1_1;
            d2[curr_iter].row(1) = t1_1.cross(d1_1);

            curr_iter++;
        }
    }
}

void ElasticJoint::prepLimbs() {
    updateJoint();
    updateRods();
}

void ElasticJoint::prepareForIteration() {
    computeTangent();
    computeTimeParallel();
    computeMaterialDirectors();
    getRefTwist();
    computeEdgeLen();
    computeKappa();
}

void ElasticJoint::setMass() {
    mass = 0;
    int limb_idx;
    std::shared_ptr<ElasticRod> curr_limb;
    for (int i = 0; i < ne; i++) {
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];

        mass += 0.5 * ref_len(i) * curr_limb->cross_sectional_area * curr_limb->rho;
    }
}
