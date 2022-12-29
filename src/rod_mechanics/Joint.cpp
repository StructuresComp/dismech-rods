#include "Joint.h"

Joint::Joint(int node, int limb_idx, vector<shared_ptr<elasticRod>> &m_limbs)
            : joint_node(node), joint_limb(limb_idx), limbs(m_limbs)
{
    ne = 0;
    limbs[limb_idx]->addJoint(joint_node, false);
    dt = limbs[limb_idx]->dt;
    updateConnectedNodes(joint_node, joint_limb, false);

}

void Joint::updateConnectedNodes(int node_num, int limb_idx, bool remove_dof) {
    int nv = limbs[limb_idx]->nv;
    if (node_num == 0) {
        pair<int, int> node_and_limb(1, limb_idx);
        connected_nodes.push_back(node_and_limb);
        if (remove_dof) {
            pair<int, int> node_and_limb2(0, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else if (node_num == nv-1) {
        pair<int, int> node_and_limb(nv-2, limb_idx);
        connected_nodes.push_back(node_and_limb);
        if (remove_dof) {
            pair<int, int> node_and_limb2(nv-1, limb_idx);
            replaced_nodes.push_back(node_and_limb2);
        }
        ne += 1;
    }
    else {
        pair<int, int> node_and_limb1(node_num-1, limb_idx);
        connected_nodes.push_back(node_and_limb1);
        pair<int, int> node_and_limb2(node_num+1, limb_idx);
        connected_nodes.push_back(node_and_limb2);
        if (remove_dof) {
            pair<int, int> node_and_limb3(node_num, limb_idx);
            replaced_nodes.push_back(node_and_limb3);
        }
        ne += 2;
    }
}


void Joint::updateJoint() {
    x(0) = limbs[joint_limb]->x(4*joint_node);
    x(1) = limbs[joint_limb]->x(4*joint_node+1);
    x(2) = limbs[joint_limb]->x(4*joint_node+2);
}


void Joint::updateRods() {
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (const auto& node_and_limb : replaced_nodes) {
        num_node = node_and_limb.first;
        limb_idx = node_and_limb.second;
        curr_limb = limbs[limb_idx];
        curr_limb->x(4* num_node) = limbs[joint_limb]->x(4*joint_node);
        curr_limb->x(4* num_node+1) = limbs[joint_limb]->x(4*joint_node+1);
        curr_limb->x(4* num_node+2) = limbs[joint_limb]->x(4*joint_node+2);
    }
}


void Joint::addToJoint(int node_num, int limb_idx) {
    limbs[limb_idx]->addJoint(node_num, true);
    updateConnectedNodes(node_num, limb_idx, true);
}


void Joint::setReferenceLength() {
    shared_ptr<elasticRod> curr_limb;
    int num_node;
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


void Joint::computeTangent() {
    // NOTE: all tangents are pointing toward the joint
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];

        tangents.row(i) = x - curr_limb->x.segment(4*num_node, 3);
        tangents.row(i) /= tangents.row(i).norm();
    }
}


void Joint::getRefandMaterialDirectors() {
    shared_ptr<elasticRod> curr_limb;
    int num_node;
    int limb_idx;
    // TODO: check that num_node will always work
    for (int i = 0; i < ne; i++) {
        num_node = connected_nodes[i].first;
        limb_idx = connected_nodes[i].second;
        curr_limb = limbs[limb_idx];
        d1.row(i) = curr_limb->d1.row(num_node);
        d2.row(i) = curr_limb->d2.row(num_node);
        m1.row(i) = curr_limb->m1.row(num_node);
        m2.row(i) = curr_limb->m2.row(num_node);
    }
}


//void Joint::getRefDirectors() {
//    shared_ptr<elasticRod> curr_limb;
//    int num_node;
//    int limb_idx;
//    // TODO: check that num_node will always work
//    for (int i = 0; i < ne; i++) {
//        num_node = connected_nodes[i].first;
//        limb_idx = connected_nodes[i].second;
//        curr_limb = limbs[limb_idx];
//        d1.row(i) = curr_limb->d1.row(num_node);
//        d2.row(i) = curr_limb->d2.row(num_node);
//    }
//}
//
//
//void Joint::computeMaterialDirectors() {
//    double cs, ss;
//    double angle;
//    shared_ptr<elasticRod> curr_limb;
//    int num_node;
//    int limb_idx;
//
//    for (int i = 0; i < ne; i++) {
//        num_node = connected_nodes[i].first;
//        limb_idx = connected_nodes[i].second;
//        curr_limb = limbs[limb_idx];
//        angle = curr_limb->x[4*num_node+3];
//        cs = cos(angle);
//        ss = sin(angle);
//
//        m1.row(i) = cs * d1.row(i) + ss * d2.row(i);
//        m2.row(i) = -ss * d1.row(i) + cs * d2.row(i);
//    }
//}


void Joint::computeKappa() {
    Vector3d t0, t1;
    Vector3d m1e, m2e, m1f, m2f;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            t0 = tangents.row(i);
            t1 = -tangents.row(j);
            kb.row(curr_iter) = 2.0 * t0.cross(t1) / (1.0 + t0.dot(t1));
            curr_iter++;
        }
    }

    curr_iter = 0;
    for (int i = 0; i < ne; i++) {
        for (int j = i + 1; j < ne; j++) {
            m1e = m1.row(i);
            m2e = m2.row(i);
            m1f = m1.row(j);
            m2f = m2.row(j);
            kappa(curr_iter, 0) = 0.5 * (kb.row(curr_iter)).dot(m2e + m2f);
            kappa(curr_iter, 1) = -0.5 * (kb.row(curr_iter)).dot(m1e + m1f);
            curr_iter++;
        }
    }
}

void Joint::parallelTransport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2)
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


void Joint::rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta)
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

double Joint::signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n)
{
    //Compute the angle between two vectors
    Vector3d w=u.cross(v);
    double angle=atan2(w.norm(),u.dot(v));
    if (n.dot(w)<0)
        return -angle;
    else
        return angle;
}


void Joint::getRefTwist() {
    Vector3d u0, u1, t0, t1, ut;
    double sgnAngle;
    int curr_iter = 0;

    for (int i = 0; i < ne; i++) {
        for (int j = i+1; j < ne; j++) {
            u0 = d1.row(i);
            u1 = d1.row(j);
            t0 = tangents.row(i);
            t1 = -tangents.row(j);
            parallelTransport(u0, t0, t1, ut);
            rotateAxisAngle(ut, t1, ref_twist_old(curr_iter));

            sgnAngle = signedAngle(ut, u1, t1);
            ref_twist(curr_iter) = ref_twist_old(curr_iter) + sgnAngle;
            curr_iter++;
        }
    }
}


void Joint::computeEdgeLen() {
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


void Joint::setup() {
    num_bending_combos = 0;
    for (int i = 0; i < ne; i++) {
        for (int j = i+1; j < ne; j++) {
            num_bending_combos++;
        }
    }

    updateJoint();
    updateRods();
    x0 = x;
    u = VectorXd::Zero(3);

    // All nodal quantities will have multiple values since there are
    // multiple possible connections to the joint
    ref_len = VectorXd(ne);
    voronoi_len = VectorXd(num_bending_combos);

    tangents = MatrixXd::Zero(ne, 3);

    d1 = MatrixXd::Zero(ne, 3);
    d2 = MatrixXd::Zero(ne, 3);
    d1_old = MatrixXd::Zero(ne, 3);
    d2_old = MatrixXd::Zero(ne, 3);
    m1 = MatrixXd::Zero(ne, 3);
    m2 = MatrixXd::Zero(ne, 3);
    ref_twist = VectorXd::Zero(num_bending_combos);
    ref_twist_old = VectorXd::Zero(num_bending_combos);

    kb = MatrixXd::Zero(num_bending_combos, 3);
    kappa = MatrixXd::Zero(num_bending_combos, 2);
    kappaBar = MatrixXd::Zero(num_bending_combos, 2);
    edge_len = VectorXd::Zero(ne);

    setMass();

    setReferenceLength();

    computeTangent();

    getRefandMaterialDirectors();

    computeKappa();

    kappaBar = kappa;

    getRefTwist();

    computeEdgeLen();

    d1_old = d1;
    d2_old = d2;
    tangents_old = tangents;
    ref_twist_old = ref_twist;

}


void Joint::computeTimeParallel()
{
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vector3d t0,t1, d1_vector;

    for (int i = 0; i < ne; i++) {
        t0 = tangents_old.row(i);
        t1 = tangents.row(i);
        parallelTransport(d1_old.row(i), t0, t1, d1_vector);

        d1.row(i) = d1_vector;
        d2.row(i) = t1.cross(d1_vector);
    }
}


void Joint::prepLimbs() {
    updateJoint();
    updateRods();
}


void Joint::prepareForIteration() {
    computeTangent();
    getRefandMaterialDirectors();  // TODO: maybe remove this later, pointless copies
//    computeTimeParallel();
    getRefTwist();
    computeEdgeLen();
    computeKappa();
}


void Joint::setMass() {
    mass = 0;
    double curr_mass;
    shared_ptr<elasticRod> curr_limb;
    for (int i = 0; i < ne; i++) {
        int node_num = connected_nodes[i].first;
        int limb_num = connected_nodes[i].second;
        curr_limb = limbs[limb_num];

        curr_mass = (curr_limb->rodLength * curr_limb->crossSectionalArea * curr_limb->rho / curr_limb->ne) / 2.0;
        mass += curr_mass;
    }
}

void Joint::updateTimeStep()
{
    prepareForIteration();

    // compute velocity
    u = (x - x0) / dt;

    // update x
    x0 = x;

    // update reference directors
    d1_old = d1;
    d2_old = d2;

    // We do not need to update m1, m2. They can be determined from theta.
    tangents_old = tangents;
    ref_twist_old = ref_twist;
}
