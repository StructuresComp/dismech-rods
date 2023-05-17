#include "elasticRod.h"

elasticRod::elasticRod(int m_limb_idx, const Vector3d &start, const Vector3d &end, int num_nodes,
                       double m_rho, double m_rodRadius, double m_dt, double m_youngM, double m_shearM)
{
    ndof = num_nodes * 4 - 1;
    nv = num_nodes;
    ne = num_nodes - 1;
    limb_idx = m_limb_idx;

    Vector3d dir = (end - start) / (num_nodes - 1);
    for (int i = 0; i < num_nodes; i++)
    {
        all_nodes.emplace_back(start + i * dir);
    }

    rodLength = (end - start).norm();

    dt = m_dt;
    youngM = m_youngM;
    shearM = m_shearM;
    rho = m_rho;
    rodRadius = m_rodRadius;

    setup();
}

// TODO: this is no longer supported, add support later if needed
elasticRod::elasticRod(MatrixXd initialNodes, MatrixXd undeformed,
                       double m_rho, double m_rodRadius, double m_dt,
                       double m_youngM, double m_shearM, double m_rodLength, VectorXd m_theta)
{
    rodLength = m_rodLength;
    nodes = initialNodes;
    nodesUndeformed = undeformed;
    nv = nodes.rows();
    ne = nv - 1;
    ndof = 3 * nv + ne;
    dt = m_dt;
    youngM = m_youngM;
    shearM = m_shearM;
    rho = m_rho;
    rodRadius = m_rodRadius;
    theta = m_theta;
}

void elasticRod::setup()
{
    x = VectorXd::Zero(ndof);
    for (int i = 0; i < nv; i++)
    {
        x(4 * i) = all_nodes[i](0);
        x(4 * i + 1) = all_nodes[i](1);
        x(4 * i + 2) = all_nodes[i](2);
        if (i < nv - 1)
        {
            x(4 * i + 3) = 0;
        }
    }
    x0 = x;

    u = VectorXd::Zero(ndof);

    // We will start off with an unconstrained system
    ncons = 0;
    uncons = ndof;
    unique_dof = ndof;
    isConstrained = new int[ndof];
    isDOFJoint = new int[ndof];
    isNodeJoint = new int[nv];
    isEdgeJoint = new int[ne];
    DOFoffsets = new int[ndof];
    for (int i = 0; i < ndof; i++)
    {
        isConstrained[i] = 0;
        isDOFJoint[i] = 0;
        DOFoffsets[i] = 0;
    }
    for (int i = 0; i < nv; i++)
    {
        isNodeJoint[i] = 0;
        pair<int, int> non_joint{i, limb_idx};
        joint_ids.push_back(non_joint);
    }
    for (int i = 0; i < ne; i++)
    {
        isEdgeJoint[i] = 0;
    }

    // Setup the map from free dofs to all dof
    unconstrainedMap = new int[uncons]; // maps xUncons to x
    fullToUnconsMap = new int[ndof];
    setupMap();

    crossSectionalArea = M_PI * rodRadius * rodRadius;

    d1 = MatrixXd::Zero(ne, 3);
    d2 = MatrixXd::Zero(ne, 3);
    d1_old = MatrixXd::Zero(ne, 3);
    d2_old = MatrixXd::Zero(ne, 3);
    m1 = MatrixXd::Zero(ne, 3);
    m2 = MatrixXd::Zero(ne, 3);
    refTwist = VectorXd(ne);

    // compute reference and voronoi lengths
    setReferenceLength();
    // set mass array
    setMass();
    // set tangent
    tangent = MatrixXd::Zero(ne, 3);
    computeTangent(x, tangent);
    // set reference directors
    computeSpaceParallel();
    // set material directors
    computeMaterialDirector();
    // compute natural curvature
    kb = MatrixXd::Zero(nv, 3);
    kappa = MatrixXd::Zero(nv, 2);
    kappaBar = MatrixXd::Zero(nv, 2);
    phi = 0;
    computeKappa();
//    computeAngle2KappaBar();
    // Reference twist
    refTwist_old = VectorXd::Zero(ne);
    getRefTwist();

    // Compute undeformed twist
    twistBar = VectorXd::Zero(ne);
    computeTwistBar();

    // compute edge length
    edgeLen = VectorXd(ne);
    computeEdgeLen();
    // compute elastic stiffness
    computeElasticStiffness();

    // values at the beginning of time step
    x0 = x;
    d1_old = d1;
    d2_old = d2;
    tangent_old = tangent;
    refTwist_old = refTwist;
}

void elasticRod::addJoint(int node_num, bool remove_dof, int joint_node, int joint_limb)
{
    if (remove_dof)
    {
        unique_dof -= 3;
        //        uncons -= 3;
        for (int i = 4 * node_num + 3; i < ndof; i++)
        {
            DOFoffsets[i] -= 3;
        }

        isDOFJoint[4 * node_num] = 1;
        isDOFJoint[4 * node_num + 1] = 1;
        isDOFJoint[4 * node_num + 2] = 1;

        if (node_num == 0)
        {
            isNodeJoint[0] = 1;
            isEdgeJoint[0] = 1;
            joint_ids[0] = pair<int, int>(joint_node, joint_limb);
        }
        else if (node_num == nv - 1)
        {
            isNodeJoint[nv - 1] = 1;
            isEdgeJoint[nv - 2] = 1;
            joint_ids[nv - 1] = pair<int, int>(joint_node, joint_limb);
        }
        else
        {
            throw runtime_error("Tried removing dofs at the mid point of an edge.");
        }
    }
    else
    {
        isDOFJoint[4 * node_num] = 2;
        isDOFJoint[4 * node_num + 1] = 2;
        isDOFJoint[4 * node_num + 2] = 2;

        // NOTE: Might be able to delete this
        if (node_num == 0)
        {
            isNodeJoint[0] = 2;
            isEdgeJoint[0] = 2;
        }
        else if (node_num == nv - 1)
        {
            isNodeJoint[nv - 1] = 2;
            isEdgeJoint[nv - 2] = 2;
        }
        else
        {
            isNodeJoint[node_num] = 2;
            isEdgeJoint[node_num - 1] = 2;
            isEdgeJoint[node_num] = 2;
        }
    }
}

void elasticRod::updateMap()
{
    ncons = 0;
    for (int i = 0; i < ndof; i++)
    {
        if (isConstrained[i] > 0 || isDOFJoint[i] == 1)
        {
            ncons++;
        }
    }
    uncons = ndof - ncons;

    delete[] unconstrainedMap;
    delete[] fullToUnconsMap;
    // Setup the map from free dofs to all dof
    unconstrainedMap = new int[uncons]; // maps xUncons to x
    fullToUnconsMap = new int[ndof];
    setupMap();
}

void elasticRod::freeVertexBoundaryCondition(int k)
{
    isConstrained[4 * k] = 0;
    isConstrained[4 * k + 1] = 0;
    isConstrained[4 * k + 2] = 0;
}

// functions to handle boundary condition
void elasticRod::setVertexBoundaryCondition(Vector3d position, int k)
{
    isConstrained[4 * k] = 1;
    isConstrained[4 * k + 1] = 1;
    isConstrained[4 * k + 2] = 1;
    // Store in the constrained dof vector
    x(4 * k) = position(0);
    x(4 * k + 1) = position(1);
    x(4 * k + 2) = position(2);
}

void elasticRod::setThetaBoundaryCondition(double desiredTheta, int k)
{
    isConstrained[4 * k + 3] = 1;
    x(4 * k + 3) = desiredTheta;
}

elasticRod::~elasticRod()
{
    delete[] isConstrained;
    delete[] unconstrainedMap;
    delete[] fullToUnconsMap;
    delete[] isDOFJoint;
    delete[] isNodeJoint;
    delete[] isEdgeJoint;
    delete[] DOFoffsets;
}

int elasticRod::getIfConstrained(int k)
{
    return isConstrained[k];
}

void elasticRod::setMass()
{
    massArray = VectorXd::Zero(ndof);

    for (int i = 0; i < nv; i++)
    {
        dm = rodLength * crossSectionalArea * rho / ne;
        if (i == 0 || i == nv - 1)
            dm = dm / 2.0;

        for (int k = 0; k < 3; k++)
        {
            massArray[4 * i + k] = dm;
        }

        if (i < nv - 1)
        {
            massArray[4 * i + 3] = rodRadius * rodRadius * dm / 2.0;
        }
    }
}

void elasticRod::setReferenceLength()
{
    // This function is only run once at sim initilization
    refLen = VectorXd(ne);
    Vector3d dx;
    for (int i = 0; i < ne; i++)
    {
        refLen(i) = rodLength / ne;
    }

    voronoiLen = VectorXd(nv);
    for (int i = 0; i < nv; i++)
    {
        if (i == 0)
            voronoiLen(i) = 0.5 * refLen(i);
        else if (i == nv - 1)
            voronoiLen(i) = 0.5 * refLen(i - 1);
        else
            voronoiLen(i) = 0.5 * (refLen(i - 1) + refLen(i));
    }
}

Vector3d elasticRod::getVertex(int k)
{
    return Vector3d(x(4 * k), x(4 * k + 1), x(4 * k + 2));
}

Vector3d elasticRod::getPreVertex(int k)
{
    return Vector3d(x0(4 * k), x0(4 * k + 1), x0(4 * k + 2));
}

Vector3d elasticRod::getVelocity(int k)
{
    return Vector3d(u(4 * k), u(4 * k + 1), u(4 * k + 2));
}

Vector3d elasticRod::getTangent(int k)
{
    return tangent.row(k);
}

double elasticRod::getTheta(int k)
{
    return x(4 * k + 3);
}

void elasticRod::computeTimeParallel()
{
    // Use old versions of (d1, d2, tangent) to get new d1, d2
    Vector3d t0, t1, d1_vector;

    for (int i = 0; i < ne; i++)
    {
        t0 = tangent_old.row(i);
        t1 = tangent.row(i);
        parallelTansport(d1_old.row(i), t0, t1, d1_vector);

        d1.row(i) = d1_vector;
        d2.row(i) = t1.cross(d1_vector);
    }
}

void elasticRod::computeTangent(const VectorXd &xLocal, MatrixXd &tangentLocal)
{
    for (int i = 0; i < ne; i++)
    {
        tangentLocal.row(i) = xLocal.segment(4 * (i + 1), 3) - xLocal.segment(4 * i, 3);
        tangentLocal.row(i) = tangentLocal.row(i) / (tangentLocal.row(i)).norm();
    }
}

void elasticRod::parallelTansport(const Vector3d &d1_1, const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2)
{
    Vector3d b;
    Vector3d n1, n2;

    b = t1.cross(t2);

    if (b.norm() == 0)
        d1_2 = d1_1;
    else
    {
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

void elasticRod::computeSpaceParallel()
{
    // This function is only called once
    Vector3d t0, t1, d1Tmp;
    Vector3d a, b, c, d;

    t0 = tangent.row(0);
    t1 << 0, 0, -1;
    d1Tmp = t0.cross(t1);

    if (fabs(d1Tmp.norm()) < 1.0e-6)
    {
        t1 << 0, 1, 0;
        d1Tmp = t0.cross(t1);
    }

    d1.row(0) = d1Tmp;
    d2.row(0) = t0.cross(d1Tmp);

    for (int i = 0; i < ne - 1; i++)
    {
        a = d1.row(i);
        b = tangent.row(i);
        c = tangent.row(i + 1);
        parallelTansport(a, b, c, d);
        d1.row(i + 1) = d;
        d2.row(i + 1) = c.cross(d);
    }
}

void elasticRod::computeMaterialDirector()
{
    // TODOder: take out cs, ss out of the function and declare them as privates in elasticRod class
    double cs, ss;
    double angle;

    for (int i = 0; i < ne; i++)
    {
        angle = x(4 * i + 3);
        cs = cos(angle);
        ss = sin(angle);
        m1.row(i) = cs * d1.row(i) + ss * d2.row(i);
        m2.row(i) = -ss * d1.row(i) + cs * d2.row(i);
    }
}

void elasticRod::computeKappa()
{
    // We know the tangent, m1, m2. Compute kappa using them
    Vector3d t0, t1;
    Vector3d m1e, m2e, m1f, m2f;

    for (int i = 1; i < ne; i++)
    {
        t0 = tangent.row(i - 1);
        t1 = tangent.row(i);
        kb.row(i) = 2.0 * t0.cross(t1) / (1.0 + t0.dot(t1));
    }

    for (int i = 1; i < ne; i++)
    {
        m1e = m1.row(i - 1);
        m2e = m2.row(i - 1);
        m1f = m1.row(i);
        m2f = m2.row(i);
        kappa(i, 0) = 0.5 * (kb.row(i)).dot(m2e + m2f);
        kappa(i, 1) = -0.5 * (kb.row(i)).dot(m1e + m1f);
    }
}

void elasticRod::updatePhi(double phi_value)
{
    phi = phi_value;
}

void elasticRod::computeAngle2KappaBar()
{
    // We know the tangent, m1, m2. Compute kappa using them
    Vector3d t0, t1;
    Vector3d m1e, m2e, m1f, m2f;
    double k;

    for (int i = 1; i < ne; i++)
    {
        t0 = tangent.row(i - 1);
        t1 = tangent.row(i);

        k = std::tan(phi / ne * (PI / 180));
        kb.row(i) = k * t0.cross(t1) / (t0.cross(t1).norm());
    }

    for (int i = 1; i < ne; i++)
    {
        m1e = m1.row(i - 1);
        m2e = m2.row(i - 1);
        m1f = m1.row(i);
        m2f = m2.row(i);
        kappaBar(i, 0) = 0.5 * (kb.row(i)).dot(m2e + m2f);
        kappaBar(i, 1) = -0.5 * (kb.row(i)).dot(m1e + m1f);
        // cout << kappaBar(i, 0) << endl;
        // cout << kappaBar(i, 1) << endl;
    }
}

void elasticRod::getRefTwist()
{
    Vector3d u0, u1, t0, t1, ut;
    double sgnAngle;

    for (int i = 1; i < ne; i++)
    {
        u0 = d1.row(i - 1);
        u1 = d1.row(i);
        t0 = tangent.row(i - 1);
        t1 = tangent.row(i);

        parallelTansport(u0, t0, t1, ut);
        rotateAxisAngle(ut, t1, refTwist_old(i));

        sgnAngle = signedAngle(ut, u1, t1);
        refTwist(i) = refTwist_old(i) + sgnAngle;
    }
}

void elasticRod::computeTwistBar()
{
    double theta_i, theta_f;
    for (int i = 1; i < ne; i++)
    {
        theta_i = x(4 * (i - 1) + 3);
        theta_f = x(4 * i + 3);
        twistBar(i) = theta_f - theta_i + refTwist(i);
    }
}

void elasticRod::computeEdgeLen()
{
    for (int i = 0; i < ne; i++)
    {
        edgeLen[i] = (x.segment(4 * (i + 1), 3) - x.segment(4 * i, 3)).norm();
    }
}

double elasticRod::signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n)
{
    // Compute the angle between two vectors
    Vector3d w = u.cross(v);
    double angle = atan2(w.norm(), u.dot(v));
    if (n.dot(w) < 0)
        return -angle;
    else
        return angle;
}

void elasticRod::rotateAxisAngle(Vector3d &v, const Vector3d &z, const double &theta)
{
    // Compute the vector when it rotates along another vector into certain angle
    if (theta != 0) // if theta=0, v = v
    {
        double cs, ss;
        cs = cos(theta);
        ss = sin(theta);
        v = cs * v + ss * z.cross(v) + z.dot(v) * (1.0 - cs) * z;
    }
}

void elasticRod::setupMap()
{
    int c = 0;
    for (int i = 0; i < ndof; i++)
    {
        if (isConstrained[i] == 0 && isDOFJoint[i] != 1)
        {
            unconstrainedMap[c] = i;
            fullToUnconsMap[i] = c;
            c++;
        }
    }
}

void elasticRod::computeElasticStiffness()
{
    EI = (youngM * M_PI * rodRadius * rodRadius * rodRadius * rodRadius) / 4;
    EA = youngM * M_PI * rodRadius * rodRadius;
    GJ = (shearM * M_PI * rodRadius * rodRadius * rodRadius * rodRadius) / 2;
}

void elasticRod::prepareForIteration()
{
    computeTangent(x, tangent);
    computeTimeParallel();
    getRefTwist();
    computeMaterialDirector();
    computeEdgeLen();
    // KappaBar will be updated by the controller
//    computeAngle2KappaBar();
    computeKappa();
}

void elasticRod::updateNewtonX(double *dx, int offset, double alpha)
{
    int ind;
    for (int c = 0; c < uncons; c++)
    {
        ind = unconstrainedMap[c];
        x[ind] -= alpha * dx[offset + c];
    }
}

void elasticRod::updateTimeStep()
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
    tangent_old = tangent;
    refTwist_old = refTwist;
}

void elasticRod::updateGuess(double weight)
{
    int ind;
    for (int c = 0; c < uncons; c++)
    {
        ind = unconstrainedMap[c];
        x[ind] = x0[ind] + weight * u[ind] * dt;
    }
}
