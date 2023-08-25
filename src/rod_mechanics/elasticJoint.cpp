#include "elasticJoint.h"

elasticJoint::elasticJoint(int node, int limb_idx, const vector<shared_ptr<elasticRod>> &m_limbs)
            : joint_node(node), joint_limb(limb_idx), limbs(m_limbs)
{
    ne = 0;
    limbs[limb_idx]->addJoint(joint_node, false, 0, 0);
    updateConnectedNodes(joint_node, joint_limb, false);
}

void elasticJoint::setup() {
    num_bending_combos = 0;
    int n1, n2;
    int sgn1, sgn2;
    int theta1_i, theta2_i;
    for (int i = 0; i < ne; i++) {
        n1 = connected_nodes[i].first;
        bending_twist_signs[i] == 1 ? sgn1 = 1 : sgn1 = -1;
        bending_twist_signs[i] == 1 ? theta1_i = 4*n1+3 : theta1_i = 4*n1-1;
        for (int j = i+1; j < ne; j++) {
            n2 = connected_nodes[j].first;
            bending_twist_signs[j] == 1 ? sgn2 = -1 : sgn2 = 1;
            bending_twist_signs[j] == 1 ? theta2_i = 4*n2+3 : theta2_i = 4*n2-1;
            sgns.emplace_back(Vector2i(sgn1, sgn2));
            theta_inds.emplace_back(Vector2i(theta1_i, theta2_i));
            num_bending_combos++;
        }
    }

    updateJoint();
    updateRods();
    x0 = x;
    u0 = u;

    // All nodal quantities will have multiple values since there are
    // multiple possible connections to the joint
    ref_len = VectorXd(ne);
    voronoi_len = VectorXd(num_bending_combos);

    tangents = MatrixXd::Zero(ne, 3);

    for (int i = 0; i < num_bending_combos; i++) {
        d1.emplace_back(MatrixXd::Zero(2, 3));
        d2.emplace_back(MatrixXd::Zero(2, 3));
        d1_old.emplace_back(MatrixXd::Zero(2, 3));
        d2_old.emplace_back(MatrixXd::Zero(2, 3));
        m1.emplace_back(MatrixXd::Zero(2, 3));
        m2.emplace_back(MatrixXd::Zero(2, 3));
    }
    ref_twist = VectorXd::Zero(num_bending_combos);
    ref_twist_old = VectorXd::Zero(num_bending_combos);

    kb = MatrixXd::Zero(num_bending_combos, 3);
    kappa = MatrixXd::Zero(num_bending_combos, 2);
    twistBar = VectorXd::Zero(num_bending_combos);
    edge_len = VectorXd::Zero(ne);

    setMass();

    setReferenceLength();

    computeTangent();

    createReferenceDirectors();

    computeMaterialDirectors();

    computeKappa();

    kappaBar = kappa;

    getRefTwist();

    computeTwistBar();

    computeEdgeLen();

    d1_old = d1;
    d2_old = d2;
    tangents_old = tangents;
    ref_twist_old = ref_twist;

}

void elasticJoint::updateConnectedNodes(int node_num, int limb_idx, bool remove_dof) {
    int nv = limbs[limb_idx]->nv;
    if (node_num == 0) {
        pair<int, int> node_and_limb(1, limb_idx);
        connected_nodes.push_back(node_and_limb);
        bending_twist_signs.push_back(-1);
        if (remove_dof) {
            pair<int, int> node_and_limb2(0, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else if (node_num == nv-1) {
        pair<int, int> node_and_limb(nv-2, limb_idx);
        connected_nodes.push_back(node_and_limb);
        bending_twist_signs.push_back(1);
        if (remove_dof) {
            pair<int, int> node_and_limb2(nv-1, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else {
        pair<int, int> node_and_limb1(node_num-1, limb_idx);
        connected_nodes.push_back(node_and_limb1);
        bending_twist_signs.push_back(1);
        pair<int, int> node_and_limb2(node_num+1, limb_idx);
        connected_nodes.push_back(node_and_limb2);
        bending_twist_signs.push_back(-1);
        if (remove_dof) {
            pair<int, int> node_and_limb3(node_num, limb_idx);
            replaced_nodes.push_back(node_and_limb3);
        }
        ne += 2;
    }
}


void elasticJoint::updateJoint() {
    x(0) = limbs[joint_limb]->x(4*joint_node);
    x(1) = limbs[joint_limb]->x(4*joint_node+1);
    x(2) = limbs[joint_limb]->x(4*joint_node+2);
    x0(0) = limbs[joint_limb]->x0(4*joint_node);
    x0(1) = limbs[joint_limb]->x0(4*joint_node+1);
    x0(2) = limbs[joint_limb]->x0(4*joint_node+2);
    u(0) = limbs[joint_limb]->u(4*joint_node);
    u(1) = limbs[joint_limb]->u(4*joint_node+1);
    u(2) = limbs[joint_limb]->u(4*joint_node+2);
    u0(0) = limbs[joint_limb]->u0(4*joint_node);
    u0(1) = limbs[joint_limb]->u0(4*joint_node+1);
    u0(2) = limbs[joint_limb]->u0(4*joint_node+2);
}


void elasticJoint::updateRods() {
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (const auto& node_and_limb : replaced_nodes) {
        num_node = node_and_limb.first;
        limb_idx = node_and_limb.second;
        curr_limb = limbs[limb_idx];

        curr_limb->x(4* num_node) = x(0);
        curr_limb->x(4* num_node+1) = x(1);
        curr_limb->x(4* num_node+2) = x(2);
    }
}


void elasticJoint::addToJoint(int node_num, int limb_idx) {
    limbs[limb_idx]->addJoint(node_num, true, joint_node, joint_limb);
    updateConnectedNodes(node_num, limb_idx, true);
}


void elasticJoint::setReferenceLength() {
    shared_ptr<elasticRod> curr_limb;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];
        ref_len(i) = curr_limb->rodLength / curr_limb->ne;
    }

    int curr_index = 0;
    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            voronoi_len(curr_index) = 0.5 * (ref_len(i) + ref_len(j));
            curr_index++;
        }
    }
}


void elasticJoint::computeTangent() {
    // NOTE: all tangents are pointing toward the joint
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];

        if (bending_twist_signs[i] == 1) {
            tangents.row(i) = x - curr_limb->x.segment(4*num_node, 3);
            tangents.row(i) /= tangents.row(i).norm();
        }
        else {
            tangents.row(i) = curr_limb->x.segment(4*num_node, 3) - x;
            tangents.row(i) /= tangents.row(i).norm();
        }
    }
}


void elasticJoint::createReferenceDirectors() {
    Vector3d t0, t1, tmp1, tmp2;
    int curr_iter = 0;
    int sgn1, sgn2;
    for (int i = 0; i < ne; i++) {
        for (int j = i+1; j < ne; j++) {
            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            t0 = tangents.row(i) * sgn1;
            t1 = tangents.row(j) * sgn2;
            tmp1 << 0, 0, -1;
            tmp2 = t0.cross(tmp1);

            if (fabs(tmp2.norm()) < 1.0e-6)
            {
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

void elasticJoint::computeMaterialDirectors() {
    double cs1, ss1;
    double cs2, ss2;
    int theta1_i, theta2_i;
    double angle1;
    double angle2;
    shared_ptr<elasticRod> limb1, limb2;
    int l1, l2;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        l1 = connected_nodes[i].second;
        for (int j = i+1; j < ne; j++) {
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


void elasticJoint::computeKappa() {
    Vector3d t0, t1;
    Vector3d m1e, m2e, m1f, m2f;
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

void elasticJoint::parallelTransport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2)
{
    Vector3d b;
    Vector3d n1,n2;

    b=t1.cross(t2);

    if(b.norm()==0)
        d1_2=d1_1;
    else
    {
        b = b / b.norm();
        b = b - b.dot(t1) * t1;
        b = b / b.norm();
        b = b - b.dot(t1) * t2;
        b = b / b.norm();

        n1=t1.cross(b);
        n2=t2.cross(b);
        d1_2=d1_1.dot(t1)*t2+d1_1.dot(n1)*n2+d1_1.dot(b)*b;
        d1_2=d1_2-d1_2.dot(t2)*t2;
        d1_2=d1_2/d1_2.norm();
    }
}


void elasticJoint::rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta)
{
    //Compute the vector when it rotates along another vector into certain angle
    if (theta!=0) // if theta=0, v = v
    {
        double cs,ss;
        cs=cos(theta);
        ss=sin(theta);
        v=cs*v+ss*z.cross(v)+z.dot(v)*(1.0-cs)*z;
    }
}

double elasticJoint::signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n)
{
    //Compute the angle between two vectors
    Vector3d w=u.cross(v);
    double angle=atan2(w.norm(),u.dot(v));
    if (n.dot(w)<0)
        return -angle;
    else
        return angle;
}


void elasticJoint::getRefTwist() {
    Vector3d u0, u1, t0, t1, ut;
    double sgnAngle;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i+1; j < ne; j++) {
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

void elasticJoint::computeTwistBar()
{
    double theta_i, theta_f;
    int l1, l2;
    int theta1_i, theta2_i;
    int sgn1, sgn2;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        l1 = connected_nodes[i].second;
        for (int j = i+1; j < ne; j++) {
            l2 = connected_nodes[j].second;

            sgn1 = sgns[curr_iter][0];
            sgn2 = sgns[curr_iter][1];
            theta1_i = theta_inds[curr_iter][0];
            theta2_i = theta_inds[curr_iter][1];

            theta_i = limbs[l1]->x[theta1_i] * sgn1;
            theta_f = limbs[l2]->x[theta2_i] * sgn2;
            twistBar(curr_iter) = theta_f - theta_i + ref_twist(curr_iter);
            curr_iter++;
        }
    }
}


void elasticJoint::computeEdgeLen() {
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];
        edge_len[i] = (x - curr_limb->x.segment(4*num_node, 3)).norm();
    }
}


void elasticJoint::computeTimeParallel()
{
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vector3d t0_0, t1_0, t0_1, t1_1, d1_0, d1_1;
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


void elasticJoint::prepLimbs() {
    updateJoint();
    updateRods();
}


void elasticJoint::prepareForIteration() {
    computeTangent();
    computeTimeParallel();
    computeMaterialDirectors();
    getRefTwist();
    computeEdgeLen();
    computeKappa();
}


void elasticJoint::setMass() {
    mass = 0;
    double curr_mass;
    shared_ptr<elasticRod> curr_limb;
    for (int i = 0; i < ne; i++) {
        int limb_num = connected_nodes[i].second;
        curr_limb = limbs[limb_num];

        curr_mass = (curr_limb->rodLength * curr_limb->crossSectionalArea * curr_limb->rho / curr_limb->ne) / 2.0;
        mass += curr_mass;
    }
}


// NOTE: don't use this, because this will be different depending on integration scheme
//void elasticJoint::updateTimeStep()
//{
//    prepareForIteration();
//
//    // compute velocity
//    u = (x - x0) / dt;
//
//    // update x
//    x0 = x;
//
//    // update reference directors
//    d1_old = d1;
//    d2_old = d2;
//
//    // We do not need to update m1, m2. They can be determined from theta.
//    tangents_old = tangents;
//    ref_twist_old = ref_twist;
//}
