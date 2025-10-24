#include "elastic_rod.h"

ElasticRod::ElasticRod(int limb_idx, const Vec3& start, const Vec3& end, int num_nodes, double rho,
                       double rod_radius, double youngs_modulus, double poisson_ratio, double mu,
                       uint16_t col_group)
    : limb_idx(limb_idx), ndof(num_nodes * 4 - 1), nv(num_nodes), ne(num_nodes - 1), rho(rho),
      rod_radius(rod_radius), youngM(youngs_modulus), poisson_ratio(poisson_ratio), mu(mu),
      col_group(col_group) {
    std::vector<Vec3> nodes;
    Vec3 dir = (end - start) / (num_nodes - 1);
    for (int i = 0; i < num_nodes; i++) {
        nodes.emplace_back(start + i * dir);
    }

    rod_length = (end - start).norm();

    setup(nodes);
}

ElasticRod::ElasticRod(int limb_idx, const std::vector<Vec3>& nodes, double rho, double rod_radius,
                       double youngs_modulus, double poisson_ratio, double mu, uint16_t col_group)
    : limb_idx(limb_idx), ndof(nodes.size() * 4 - 1), nv(nodes.size()), ne(nodes.size() - 1),
      rho(rho), rod_radius(rod_radius), youngM(youngs_modulus), poisson_ratio(poisson_ratio),
      mu(mu), col_group(col_group) {
    rod_length = 0;
    for (int i = 1; i < nv; i++) {
        rod_length += (nodes[i] - nodes[i - 1]).norm();
    }

    setup(nodes);
}

void ElasticRod::setup(const std::vector<Vec3>& nodes) {
    x = VecX::Zero(ndof);
    for (int i = 0; i < nv; i++) {
        x(4 * i) = nodes[i](0);
        x(4 * i + 1) = nodes[i](1);
        x(4 * i + 2) = nodes[i](2);
        if (i < nv - 1) {
            x(4 * i + 3) = 0;
        }
    }
    x0 = x;

    u = VecX::Zero(ndof);
    u0 = u;

    // We will start off with an unconstrained system
    ncons = 0;
    uncons = ndof;
    isConstrained = std::vector<int>(ndof, 0);
    isDOFJoint = std::vector<int>(ndof, 0);
    isNodeJoint = std::vector<int>(nv, 0);
    isEdgeJoint = std::vector<int>(ne, 0);
    for (int i = 0; i < nv; i++) {
        std::pair<int, int> non_joint{i, limb_idx};
        joint_ids.push_back(non_joint);
    }

    // Setup the map from free dofs to all dof
    unconstrainedMap = std::vector<int>(uncons, 0);  // maps xUncons to x
    fullToUnconsMap = std::vector<int>(ndof, 0);
    setupMap();

    cross_sectional_area = M_PI * rod_radius * rod_radius;

    d1 = MatX::Zero(ne, 3);
    d2 = MatX::Zero(ne, 3);
    d1_old = MatX::Zero(ne, 3);
    d2_old = MatX::Zero(ne, 3);
    m1 = MatX::Zero(ne, 3);
    m2 = MatX::Zero(ne, 3);
    ref_twist = VecX(ne);

    // compute reference and voronoi lengths
    setReferenceLength();
    // set mass array
    setMass();
    // set tangent
    tangent = MatX::Zero(ne, 3);
    computeTangent();
    // set reference directors
    computeSpaceParallel();
    // set material directors
    computeMaterialDirector();
    // compute natural curvature
    kb = MatX::Zero(nv, 3);
    kappa = MatX::Zero(nv, 2);
    computeKappa();
    kappa_bar = kappa;

    // Reference twist
    ref_twist_old = VecX::Zero(ne);
    getRefTwist();

    // Compute undeformed twist
    twist_bar = VecX::Zero(ne);
    computeTwistBar();

    // compute edge length
    edge_len = VecX(ne);
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
            joint_ids[0] = std::pair<int, int>(joint_node, joint_limb);
        }
        else if (node_num == nv - 1) {
            isNodeJoint[nv - 1] = 1;
            isEdgeJoint[nv - 2] = 1;
            joint_ids[nv - 1] = std::pair<int, int>(joint_node, joint_limb);
        }
        else {
            throw std::runtime_error("Tried removing dofs at the mid point of an edge.");
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

    // Setup the map from free dofs to all dof
    unconstrainedMap.clear();
    unconstrainedMap.resize(uncons, 0);
    fullToUnconsMap.clear();
    fullToUnconsMap.resize(ndof, 0);
    setupMap();
}

void ElasticRod::freeVertexBoundaryCondition(int k) {
    isConstrained[4 * k] = 0;
    isConstrained[4 * k + 1] = 0;
    isConstrained[4 * k + 2] = 0;
}

// functions to handle boundary condition
void ElasticRod::setVertexBoundaryCondition(Vec3 position, int k) {
    isConstrained[4 * k] = 1;
    isConstrained[4 * k + 1] = 1;
    isConstrained[4 * k + 2] = 1;
    // Store in the constrained dof std::vector
    x(4 * k) = position(0);
    x(4 * k + 1) = position(1);
    x(4 * k + 2) = position(2);
}

void ElasticRod::setThetaBoundaryCondition(double desired_theta, int k) {
    isConstrained[4 * k + 3] = 1;
    x(4 * k + 3) = desired_theta;
}

ElasticRod::~ElasticRod() = default;

int ElasticRod::getIfConstrained(int k) const {
    return isConstrained[k];
}

void ElasticRod::setMass() {
    double dm;
    mass_array = VecX::Zero(ndof);

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
    ref_len = VecX(ne);
    for (int i = 0; i < ne; i++) {
        ref_len(i) = (x.segment(4 * (i + 1), 3) - x.segment(4 * i, 3)).norm();
    }

    voronoi_len = VecX(nv);
    for (int i = 0; i < nv; i++) {
        if (i == 0)
            voronoi_len(i) = 0.5 * ref_len(i);
        else if (i == nv - 1)
            voronoi_len(i) = 0.5 * ref_len(i - 1);
        else
            voronoi_len(i) = 0.5 * (ref_len(i - 1) + ref_len(i));
    }
}

Vec3 ElasticRod::getVertex(int k) {
    return x.segment<3>(4 * k);
}

MatXN<3> ElasticRod::getVertices() {
    MatXN<3> vertices(nv, 3);

    for (int i = 0; i < nv; i++) {
        vertices.row(i) = x.segment<3>(4 * i);
    }
    return vertices;
}

Vec3 ElasticRod::getPreVertex(int k) {
    return x0.segment<3>(4 * k);
}

Vec3 ElasticRod::getVelocity(int k) {
    return u.segment<3>(4 * k);
}

MatXN<3> ElasticRod::getVelocities() {
    MatXN<3> velocities(nv, 3);

    for (int i = 0; i < nv; i++) {
        velocities.row(i) = u.segment<3>(4 * i);
    }
    return velocities;
}

Vec3 ElasticRod::getTangent(int k) {
    return tangent.row(k);
}

double ElasticRod::getTheta(int k) {
    return x(4 * k + 3);
}

VecX ElasticRod::getThetas() {
    VecX thetas(ne);

    for (int i = 0; i < ne; i++) {
        thetas(i) = x(4 * i + 3);
    }
    return thetas;
}

void ElasticRod::computeTimeParallel() {
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vec3 t0, t1, d1_vector;

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

void ElasticRod::parallelTransport(const Vec3& d1_1, const Vec3& t1, const Vec3& t2, Vec3& d1_2) {
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

void ElasticRod::computeSpaceParallel() {
    // This function is only called once
    Vec3 t0, t1, d1Tmp;
    Vec3 a, b, c, d;

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
    Vec3 t0, t1;
    Vec3 m1e, m2e, m1f, m2f;

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
    Vec3 u0, u1, t0, t1, ut;
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

double ElasticRod::signedAngle(const Vec3& u, const Vec3& v, const Vec3& n) {
    // Compute the angle between two vectors
    Vec3 w = u.cross(v);
    double angle = atan2(w.norm(), u.dot(v));
    if (n.dot(w) < 0)
        return -angle;
    else
        return angle;
}

void ElasticRod::rotateAxisAngle(Vec3& v, const Vec3& z, const double& theta) {
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

void ElasticRod::enable2DSim() {
    for (int i = 0; i < ne; i++) {
        isConstrained[4 * i + 1] = 1;
        isConstrained[4 * i + 3] = 1;
    }
    isConstrained[4 * ne + 1] = 1;
}

Vec3 ElasticRod::getM1(int i) {
    return m1.row(i);
}

Vec3 ElasticRod::getM2(int i) {
    return m2.row(i);
}
