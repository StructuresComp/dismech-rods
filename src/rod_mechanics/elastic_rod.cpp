#include "elastic_rod.h"

ElasticRod::ElasticRod(int limb_idx, const Vector3d& start, const Vector3d& end, int num_nodes,
                       double rho, double rod_radius, double youngs_modulus, double poisson_ratio,
                       double mu)
    : limb_idx(limb_idx), ndof(num_nodes * 4 - 1), nv(num_nodes), ne(num_nodes - 1), rho(rho),
      rod_radius(rod_radius), youngM(youngs_modulus), poisson_ratio(poisson_ratio), mu(mu) {
    vector<Vector3d> nodes;
    Vector3d dir = (end - start) / (num_nodes - 1);
    for (int i = 0; i < num_nodes; i++) {
        nodes.emplace_back(start + i * dir);
    }

    rod_length = (end - start).norm();

    setup(nodes);
}

ElasticRod::ElasticRod(int limb_idx, const vector<Vector3d>& nodes, double rho, double rod_radius,
                       double youngs_modulus, double poisson_ratio, double mu)
    : limb_idx(limb_idx), ndof(nodes.size() * 4 - 1), nv(nodes.size()), ne(nodes.size() - 1),
      rho(rho), rod_radius(rod_radius), youngM(youngs_modulus), poisson_ratio(poisson_ratio),
      mu(mu) {
    rod_length = 0;
    for (int i = 1; i < nv; i++) {
        rod_length += (nodes[i] - nodes[i - 1]).norm();
    }

    setup(nodes);
}

void ElasticRod::setup(const vector<Vector3d>& nodes) {
    x = VectorXd::Zero(ndof);
    for (int i = 0; i < nv; i++) {
        x(4 * i) = nodes[i](0);
        x(4 * i + 1) = nodes[i](1);
        x(4 * i + 2) = nodes[i](2);
        if (i < nv - 1) {
            x(4 * i + 3) = 0;
        }
    }
    x0 = x;

    u = VectorXd::Zero(ndof);
    u0 = u;

    // We will start off with an unconstrained system
    ncons = 0;
    uncons = ndof;
    isConstrained = new int[ndof];
    isDOFJoint = new int[ndof];
    isNodeJoint = new int[nv];
    isEdgeJoint = new int[ne];
    for (int i = 0; i < ndof; i++) {
        isConstrained[i] = 0;
        isDOFJoint[i] = 0;
    }
    for (int i = 0; i < nv; i++) {
        isNodeJoint[i] = 0;
        pair<int, int> non_joint{i, limb_idx};
        joint_ids.push_back(non_joint);
    }
    for (int i = 0; i < ne; i++) {
        isEdgeJoint[i] = 0;
    }

    // Setup the map from free dofs to all dof
    unconstrainedMap = new int[uncons];  // maps xUncons to x
    fullToUnconsMap = new int[ndof];
    setupMap();

    cross_sectional_area = M_PI * rod_radius * rod_radius;

    d1 = MatrixXd::Zero(ne, 3);
    d2 = MatrixXd::Zero(ne, 3);
    d1_old = MatrixXd::Zero(ne, 3);
    d2_old = MatrixXd::Zero(ne, 3);
    m1 = MatrixXd::Zero(ne, 3);
    m2 = MatrixXd::Zero(ne, 3);
    ref_twist = VectorXd(ne);

    // compute reference and voronoi lengths
    setReferenceLength();
    // set mass array
    setMass();
    // set tangent
    tangent = MatrixXd::Zero(ne, 3);
    computeTangent();
    // set reference directors
    computeSpaceParallel();
    // set material directors
    computeMaterialDirector();
    // compute natural curvature
    kb = MatrixXd::Zero(nv, 3);
    kappa = MatrixXd::Zero(nv, 2);
    computeKappa();
    kappa_bar = kappa;

    // Reference twist
    ref_twist_old = VectorXd::Zero(ne);
    getRefTwist();

    // Compute undeformed twist
    twist_bar = VectorXd::Zero(ne);
    computeTwistBar();

    // compute edge length
    edge_len = VectorXd(ne);
    computeEdgeLen();
    // compute elastic stiffness
    computeElasticStiffness();

    // values at the beginning of time step
    d1_old = d1;
    d2_old = d2;
    tangent_old = tangent;
    ref_twist_old = ref_twist;
}

void ElasticRod::addJoint(int node_num, bool attach_to_joint, int joint_node, int joint_limb) {
    if (attach_to_joint) {
        isDOFJoint[4 * node_num] = 1;
        isDOFJoint[4 * node_num + 1] = 1;
        isDOFJoint[4 * node_num + 2] = 1;

        if (node_num == 0) {
            isNodeJoint[0] = 1;
            isEdgeJoint[0] = 1;
            joint_ids[0] = pair<int, int>(joint_node, joint_limb);
        }
        else if (node_num == nv - 1) {
            isNodeJoint[nv - 1] = 1;
            isEdgeJoint[nv - 2] = 1;
            joint_ids[nv - 1] = pair<int, int>(joint_node, joint_limb);
        }
        else {
            throw runtime_error("Tried removing dofs at the mid point of an edge.");
        }
    }
    else {
        isDOFJoint[4 * node_num] = 2;
        isDOFJoint[4 * node_num + 1] = 2;
        isDOFJoint[4 * node_num + 2] = 2;

        // NOTE: Might be able to delete this
        if (node_num == 0) {
            isNodeJoint[0] = 2;
            isEdgeJoint[0] = 2;
        }
        else if (node_num == nv - 1) {
            isNodeJoint[nv - 1] = 2;
            isEdgeJoint[nv - 2] = 2;
        }
        else {
            isNodeJoint[node_num] = 2;
            isEdgeJoint[node_num - 1] = 2;
            isEdgeJoint[node_num] = 2;
        }
    }
}

void ElasticRod::updateMap() {
    ncons = 0;
    for (int i = 0; i < ndof; i++) {
        if (isConstrained[i] > 0 || isDOFJoint[i] == 1) {
            ncons++;
        }
    }
    uncons = ndof - ncons;

    delete[] unconstrainedMap;
    delete[] fullToUnconsMap;
    // Setup the map from free dofs to all dof
    unconstrainedMap = new int[uncons];  // maps xUncons to x
    fullToUnconsMap = new int[ndof];
    setupMap();
}

void ElasticRod::freeVertexBoundaryCondition(int k) {
    isConstrained[4 * k] = 0;
    isConstrained[4 * k + 1] = 0;
    isConstrained[4 * k + 2] = 0;
}

// functions to handle boundary condition
void ElasticRod::setVertexBoundaryCondition(Vector3d position, int k) {
    isConstrained[4 * k] = 1;
    isConstrained[4 * k + 1] = 1;
    isConstrained[4 * k + 2] = 1;
    // Store in the constrained dof vector
    x(4 * k) = position(0);
    x(4 * k + 1) = position(1);
    x(4 * k + 2) = position(2);
}

void ElasticRod::setThetaBoundaryCondition(double desired_theta, int k) {
    isConstrained[4 * k + 3] = 1;
    x(4 * k + 3) = desired_theta;
}

ElasticRod::~ElasticRod() {
    delete[] isConstrained;
    delete[] unconstrainedMap;
    delete[] fullToUnconsMap;
    delete[] isDOFJoint;
    delete[] isNodeJoint;
    delete[] isEdgeJoint;
}

int ElasticRod::getIfConstrained(int k) const {
    return isConstrained[k];
}

void ElasticRod::setMass() {
    double dm;
    mass_array = VectorXd::Zero(ndof);

    for (int i = 0; i < nv; i++) {
        dm = 0.5 * cross_sectional_area * rho;

        if (i == 0)
            dm *= ref_len(i);
        else if (i == nv - 1)
            dm *= ref_len(i - 1);
        else
            dm *= (ref_len(i - 1) + ref_len(i));

        for (int k = 0; k < 3; k++) {
            mass_array[4 * i + k] = dm;
        }

        if (i < nv - 1) {
            mass_array[4 * i + 3] = rod_radius * rod_radius * dm / 2.0;
        }
    }
}

void ElasticRod::setReferenceLength() {
    // This function is only run once at sim initialization.
    ref_len = VectorXd(ne);
    for (int i = 0; i < ne; i++) {
        ref_len(i) = (x.segment(4 * (i + 1), 3) - x.segment(4 * i, 3)).norm();
    }

    voronoi_len = VectorXd(nv);
    for (int i = 0; i < nv; i++) {
        if (i == 0)
            voronoi_len(i) = 0.5 * ref_len(i);
        else if (i == nv - 1)
            voronoi_len(i) = 0.5 * ref_len(i - 1);
        else
            voronoi_len(i) = 0.5 * (ref_len(i - 1) + ref_len(i));
    }
}

Vector3d ElasticRod::getVertex(int k) {
    return x.segment<3>(4 * k);
}

MatrixX3d ElasticRod::getVertices() {
    MatrixX3d vertices(nv, 3);

    for (int i = 0; i < nv; i++) {
        vertices.row(i) = x.segment<3>(4 * i);
    }
    return vertices;
}

Vector3d ElasticRod::getPreVertex(int k) {
    return x0.segment<3>(4 * k);
}

Vector3d ElasticRod::getVelocity(int k) {
    return u.segment<3>(4 * k);
}

MatrixX3d ElasticRod::getVelocities() {
    MatrixX3d velocities(nv, 3);

    for (int i = 0; i < nv; i++) {
        velocities.row(i) = u.segment<3>(4 * i);
    }
    return velocities;
}

Vector3d ElasticRod::getTangent(int k) {
    return tangent.row(k);
}

double ElasticRod::getTheta(int k) {
    return x(4 * k + 3);
}

VectorXd ElasticRod::getThetas() {
    VectorXd thetas(ne);

    for (int i = 0; i < ne; i++) {
        thetas(i) = x(4 * i + 3);
    }
    return thetas;
}

void ElasticRod::computeTimeParallel() {
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vector3d t0, t1, d1_vector;

    for (int i = 0; i < ne; i++) {
        t0 = tangent_old.row(i);
        t1 = tangent.row(i);
        parallelTransport(d1_old.row(i), t0, t1, d1_vector);

        d1.row(i) = d1_vector;
        d2.row(i) = t1.cross(d1_vector);
    }
}

void ElasticRod::computeTangent() {
    for (int i = 0; i < ne; i++) {
        tangent.row(i) = x.segment(4 * (i + 1), 3) - x.segment(4 * i, 3);
        tangent.row(i) = tangent.row(i) / (tangent.row(i)).norm();
    }
}

void ElasticRod::parallelTransport(const Vector3d& d1_1, const Vector3d& t1, const Vector3d& t2,
                                   Vector3d& d1_2) {
    Vector3d b;
    Vector3d n1, n2;

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

void ElasticRod::computeSpaceParallel() {
    // This function is only called once
    Vector3d t0, t1, d1Tmp;
    Vector3d a, b, c, d;

    t0 = tangent.row(0);
    t1 << 0, 0, -1;
    d1Tmp = t0.cross(t1);

    if (fabs(d1Tmp.norm()) < 1.0e-6) {
        t1 << 0, 1, 0;
        d1Tmp = t0.cross(t1);
    }

    d1.row(0) = d1Tmp;
    d2.row(0) = t0.cross(d1Tmp);

    for (int i = 0; i < ne - 1; i++) {
        a = d1.row(i);
        b = tangent.row(i);
        c = tangent.row(i + 1);
        parallelTransport(a, b, c, d);
        d1.row(i + 1) = d;
        d2.row(i + 1) = c.cross(d);
    }
}

void ElasticRod::computeMaterialDirector() {
    double cs, ss;
    double angle;

    for (int i = 0; i < ne; i++) {
        angle = x(4 * i + 3);
        cs = cos(angle);
        ss = sin(angle);
        m1.row(i) = cs * d1.row(i) + ss * d2.row(i);
        m2.row(i) = -ss * d1.row(i) + cs * d2.row(i);
    }
}

void ElasticRod::computeKappa() {
    // We know the tangent, m1, m2. Compute kappa using them
    Vector3d t0, t1;
    Vector3d m1e, m2e, m1f, m2f;

    for (int i = 1; i < ne; i++) {
        t0 = tangent.row(i - 1);
        t1 = tangent.row(i);
        kb.row(i) = 2.0 * t0.cross(t1) / (1.0 + t0.dot(t1));
    }

    for (int i = 1; i < ne; i++) {
        m1e = m1.row(i - 1);
        m2e = m2.row(i - 1);
        m1f = m1.row(i);
        m2f = m2.row(i);
        kappa(i, 0) = 0.5 * (kb.row(i)).dot(m2e + m2f);
        kappa(i, 1) = -0.5 * (kb.row(i)).dot(m1e + m1f);
    }
}

void ElasticRod::getRefTwist() {
    Vector3d u0, u1, t0, t1, ut;
    double sgnAngle;

    for (int i = 1; i < ne; i++) {
        u0 = d1.row(i - 1);
        u1 = d1.row(i);
        t0 = tangent.row(i - 1);
        t1 = tangent.row(i);

        parallelTransport(u0, t0, t1, ut);
        rotateAxisAngle(ut, t1, ref_twist_old(i));

        sgnAngle = signedAngle(ut, u1, t1);
        ref_twist(i) = ref_twist_old(i) + sgnAngle;
    }
}

void ElasticRod::computeTwistBar() {
    double theta_i, theta_f;
    for (int i = 1; i < ne; i++) {
        theta_i = x(4 * (i - 1) + 3);
        theta_f = x(4 * i + 3);
        twist_bar(i) = theta_f - theta_i + ref_twist(i);
    }
}

void ElasticRod::computeEdgeLen() {
    for (int i = 0; i < ne; i++) {
        edge_len[i] = (x.segment(4 * (i + 1), 3) - x.segment(4 * i, 3)).norm();
    }
}

double ElasticRod::signedAngle(const Vector3d& u, const Vector3d& v, const Vector3d& n) {
    // Compute the angle between two vectors
    Vector3d w = u.cross(v);
    double angle = atan2(w.norm(), u.dot(v));
    if (n.dot(w) < 0)
        return -angle;
    else
        return angle;
}

void ElasticRod::rotateAxisAngle(Vector3d& v, const Vector3d& z, const double& theta) {
    // Compute the vector when it rotates along another vector into certain
    // angle
    if (theta != 0)  // if theta=0, v = v
    {
        double cs, ss;
        cs = cos(theta);
        ss = sin(theta);
        v = cs * v + ss * z.cross(v) + z.dot(v) * (1.0 - cs) * z;
    }
}

void ElasticRod::setupMap() {
    int c = 0;
    for (int i = 0; i < ndof; i++) {
        if (isConstrained[i] == 0 && isDOFJoint[i] != 1) {
            unconstrainedMap[c] = i;
            fullToUnconsMap[i] = c;
            c++;
        }
    }
}

void ElasticRod::computeElasticStiffness() {
    shearM = youngM / (2.0 * (1.0 + poisson_ratio));
    EI = (youngM * M_PI * rod_radius * rod_radius * rod_radius * rod_radius) / 4;
    EA = youngM * M_PI * rod_radius * rod_radius;
    GJ = (shearM * M_PI * rod_radius * rod_radius * rod_radius * rod_radius) / 2;
}

void ElasticRod::prepareForIteration() {
    computeTangent();
    computeTimeParallel();
    getRefTwist();
    computeMaterialDirector();
    computeEdgeLen();
    computeKappa();
}

double ElasticRod::updateNewtonX(double* dx, int offset, double alpha) {
    int ind;
    double max_dx = 0;
    double curr_dx = 0;
    for (int c = 0; c < uncons; c++) {
        ind = unconstrainedMap[c];
        x[ind] -= alpha * dx[offset + c];

        if ((ind - 3) % 4 != 0) {  // non-theta degree of freedom
            curr_dx = abs(dx[offset + c]);
            if (curr_dx > max_dx) {
                max_dx = curr_dx;
            }
        }
    }
    return max_dx;
}

void ElasticRod::updateGuess(double weight, double dt) {
    int ind;
    for (int c = 0; c < uncons; c++) {
        ind = unconstrainedMap[c];
        x[ind] = x0[ind] + weight * u[ind] * dt;
    }
}

void ElasticRod::enable2DSim() const {
    for (int i = 0; i < ne; i++) {
        isConstrained[4 * i + 1] = 1;
        isConstrained[4 * i + 3] = 1;
    }
    isConstrained[4 * ne + 1] = 1;
}
