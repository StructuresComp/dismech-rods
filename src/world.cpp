#include "world.h"

world::world() {
    ;
}

world::world(setInput &m_inputData) {
    render = m_inputData.GetBoolOpt("render");                       // boolean
    RodLength = m_inputData.GetScalarOpt("RodLength");               // meter
    helixradius = m_inputData.GetScalarOpt("helixradius");           // meter
    gVector = m_inputData.GetVecOpt("gVector");                            // m/s^2
    maxIter = m_inputData.GetIntOpt("maxIter");                      // maximum number of iterations
    helixpitch = m_inputData.GetScalarOpt("helixpitch");             // meter
    rodRadius = m_inputData.GetScalarOpt("rodRadius");               // mete
    numVertices = m_inputData.GetIntOpt("numVertices");              // int_num
    youngM = m_inputData.GetScalarOpt("youngM");                     // Pa
    Poisson = m_inputData.GetScalarOpt("Poisson");                   // dimensionless
    deltaTime = m_inputData.GetScalarOpt("deltaTime");               // seconds
    tol = m_inputData.GetScalarOpt("tol");                           // small number like 10e-7
    stol = m_inputData.GetScalarOpt("stol");                         // small number, e.g. 0.1%
    density = m_inputData.GetScalarOpt("density");                   // kg/m^3
    viscosity = m_inputData.GetScalarOpt("viscosity");               // viscosity in Pa-s
    data_resolution = m_inputData.GetScalarOpt("dataResolution");    // time resolution for recording data
    pull_time = m_inputData.GetScalarOpt("pullTime");                // get time of pulling
    release_time = m_inputData.GetScalarOpt("releaseTime");          // get time of loosening
    wait_time = m_inputData.GetScalarOpt("waitTime");                // get time of waiting
    pull_speed = m_inputData.GetScalarOpt("pullSpeed");              // get speed of pulling
    col_limit = m_inputData.GetScalarOpt("colLimit");                // distance limit for candidate set
    delta = m_inputData.GetScalarOpt("delta");                       // distance tolerance for contact
    k_scaler = m_inputData.GetScalarOpt("kScaler");                  // constant scaler for contact stiffness
    mu = m_inputData.GetScalarOpt("mu");                             // friction coefficient
    nu = m_inputData.GetScalarOpt("nu");                             // slipping tolerance for friction
    line_search = m_inputData.GetIntOpt("lineSearch");               // flag for enabling line search
    knot_config = m_inputData.GetStringOpt("knotConfig");            // get initial knot configuration

    shearM = youngM / (2.0 * (1.0 + Poisson));                             // shear modulus

    data_rate = ceil(data_resolution / deltaTime);                         // iter resolution for recording data
    alpha = 1.0;                                                           // newton step size
    total_iters = 0;                                                       // total number of newton iterations

    totalTime = wait_time + pull_time + release_time;                      // total sim time
}

world::~world() {
    ;
}

bool world::isRender() {
    return render;
}

void world::OpenFile(ofstream &outfile, string file_type) {
    int systemRet = system("mkdir datafiles"); //make the directory
    if (systemRet == -1) {
        cout << "Error in creating directory\n";
    }

    // Open an input file named after the current time
    ostringstream file_name;
    file_name.precision(6);
    file_name << "datafiles/" << file_type;
    file_name << "_" << knot_config;
    file_name << "_dt_" << deltaTime;
    file_name << "_totalTime_" << totalTime;
    file_name << "_mu_" << mu;
    file_name << ".txt";
    outfile.open(file_name.str().c_str());
    outfile.precision(10);
}

void world::outputNodeCoordinates(ofstream &outfile) {
    if (timeStep % data_rate != 0) return;
    Vector3d curr_node;
    double curr_theta;
    for (int i = 0; i < rod->nv-1; i++) {
        curr_node = rod->getVertex(i);
        curr_theta = rod->getTheta(i);
        outfile << curr_node(0) << " " << curr_node(1) << " " <<
                curr_node(2) << " " << curr_theta << endl;
    }
    curr_node = rod->getVertex(rod->nv-1);
    outfile << curr_node(0) << " " << curr_node(1) << " " <<
            curr_node(2) << " " << 0.0 << endl;
}

void world::CloseFile(ofstream &outfile) {
    outfile.close();
}


bool world::CoutDataC(ofstream &outfile) {
    if (timeStep % data_rate != 0) return false;
    double f;
    double f1;

    f = temp.norm();
    f1 = temp1.norm();

    double end_to_end_length = (rod->getVertex(0) - rod->getVertex(rod->nv-1)).norm();

    // 2piR method. WARNING: not the most accurate method for getting knot loop radius
    double loop_circumference = RodLength - end_to_end_length;
    double radius = loop_circumference / (2. * M_PI);

    // Output pull forces here
    // Do not need to add endl here as it will be added when time spent is added
    outfile << currentTime << " " << f << " " << f1 << " " << radius << " " << end_to_end_length
            << " " << iter << " " << total_iters << " ";
    return true;
}


void world::setRodStepper() {
    // Set up geometry
//    rodGeometry();

    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0, 0), Vector3d(0, 0.05, 0), 6,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0.03, 0), Vector3d(0.10, 0.03, 0), 10,
                                            density, rodRadius, deltaTime, youngM, shearM));
    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0.03, 0), Vector3d(-0.10, 0.03, 0), 10,
                                            density, rodRadius, deltaTime, youngM, shearM));

//    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0.025, 0), Vector3d(-0.05, 0.025, 0), 15,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0.05, 0), Vector3d(0.05, 0.05, 0), 15,
//                                            density, rodRadius, deltaTime, youngM, shearM));


    joints.push_back(make_shared<Joint>(3, 0, limbs));
    joints[0]->addToJoint(0, 1);
    joints[0]->addToJoint(0, 2);



//    joints[0]->addToJoint(0, 2);
//    joints.push_back(make_shared<Joint>(14, 0, limbs));
//    joints[1]->addToJoint(0, 3);

//    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0, 0), Vector3d(0, 0.025, 0), 5,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//    limbs.push_back(make_shared<elasticRod>(Vector3d(0, 0.025, 0), Vector3d(0.0, 0.05, 0), 5,
//                                            density, rodRadius, deltaTime, youngM, shearM));
//
//    joints.push_back(make_shared<Joint>(4, 0, limbs));
//    joints[0]->addToJoint(0, 1);

    // This has to be called after joints are all set.
    for (const auto& joint : joints) joint->setup();


    // Find out the tolerance, e.g. how small is enough?
    characteristicForce = M_PI * pow(rodRadius, 4) / 4.0 * youngM / pow(RodLength, 2);
    forceTol = tol * characteristicForce;

    // set up the time stepper
    stepper = make_shared<timeStepper>(limbs);
    totalForce = stepper->getForce();
    dx = stepper->dx;

    // Set up boundary condition
    lockEdge(0, 0);
//    lockEdge(0, 1);
    updateCons();

    // declare the forces
    m_stretchForce = make_unique<elasticStretchingForce>(limbs, joints, stepper);
    m_bendingForce = make_unique<elasticBendingForce>(limbs, joints, stepper);
//    m_twistingForce = make_unique<elasticTwistingForce>(limbs, joints, stepper);
    m_inertialForce = make_unique<inertialForce>(limbs, joints, stepper);
    m_gravityForce = make_unique<externalGravityForce>(limbs, joints, stepper, gVector);
    m_dampingForce = make_unique<dampingForce>(limbs, joints, stepper, viscosity);
//    m_collisionDetector = make_shared<collisionDetector>(rod, delta, col_limit);
//    m_contactPotentialIMC = make_unique<contactPotentialIMC>(rod, stepper, m_collisionDetector, delta, k_scaler, mu, nu);

    // Allocate every thing to prepare for the first iteration
    for (const auto& joint : joints) joint->updateTimeStep();
    for (const auto& limb : limbs) limb->updateTimeStep();
//    rod->updateTimeStep();

    currentTime = 0.0;
    timeStep = 0;
}


//void world::makeJoint(int node, int limb_idx) {
//    shared_ptr<Joint> joint;
//    joint->x = Map<Vector3d>(limbs[limb_idx]->x[4*node])
////    joints.push_back(make_shared<Joint>());
//}
//
//void world::addToJoint(int joint_num, int node, int limb_idx) {
//    shared_ptr<Joint> joint = joints[joint_num];
//    shared_ptr<elasticRod> limb = limbs[limb_idx];
//
//
//}


// Setup geometry
void world::rodGeometry() {
    vertices = MatrixXd(numVertices, 3);

    ifstream myfile(("knot_configurations/" + knot_config).c_str());

    int row1 = numVertices;

    MatrixXd data = MatrixXd(row1, 4);
    double a;
    if (myfile.is_open()) {
        for (int i = 0; i < row1 * 4; i++) {
            myfile >> a;
            if (i % 4 == 0)
                data(i / 4, 0) = a;
            else if (i % 4 == 1)
                data(i / 4, 1) = a;
            else if (i % 4 == 2)
                data(i / 4, 2) = a;
            else if (i % 4 == 3)
                data(i / 4, 3) = a;
        }
    }
    theta = VectorXd::Zero(numVertices - 1);
    for (int i = 0; i < numVertices; i++) {
        vertices(i, 0) = data(i, 0);
        vertices(i, 1) = data(i, 1);
        vertices(i, 2) = data(i, 2);
    }
}


void world::rodBoundaryCondition() {
    limbs[0]->setVertexBoundaryCondition(limbs[0]->getVertex(0), 0);
    limbs[0]->setVertexBoundaryCondition(limbs[0]->getVertex(1), 1);
    limbs[0]->setThetaBoundaryCondition(0.0, 0);
}


void world::lockEdge(int edge_num, int limb_idx) {
    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(0), 0);
    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(1), 1);
    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 0);
//    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 1);

//    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 1);
//    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(2), 2);
}


void world::updateCons() {
    for (const auto& limb : limbs) limb->updateMap();
    stepper->update();
    totalForce = stepper->getForce();
    dx = stepper->dx;
}


int world::getTimeStep() {
    return timeStep;
}


void world::updateTimeStep() {
    bool solved = false;

    newtonMethod(solved);

    // calculate pull forces;
//    calculateForce();

    for (const auto& joint : joints) joint->updateTimeStep();
    for (const auto& limb : limbs) limb->updateTimeStep();
//    rod->updateTimeStep();

    printSimData();

    currentTime += deltaTime;
    timeStep++;
}

void world::calculateForce() {
    stepper->setZero();

    m_inertialForce->computeFi();
    m_stretchForce->computeFs();
    m_bendingForce->computeFb();
//    m_twistingForce->computeFt();
    m_gravityForce->computeFg();
    m_dampingForce->computeFd();

    temp[0] = stepper->force[0] + stepper->force[4];
    temp[1] = stepper->force[1] + stepper->force[5];
    temp[2] = stepper->force[2] + stepper->force[6];

    temp1[0] = stepper->force[rod->ndof - 3] + stepper->force[rod->ndof - 7];
    temp1[1] = stepper->force[rod->ndof - 2] + stepper->force[rod->ndof - 6];
    temp1[2] = stepper->force[rod->ndof - 1] + stepper->force[rod->ndof - 5];
}


bool world::pulling() {
    return currentTime > wait_time;
}


void world::newtonDamper() {
    if (iter < 10)
        alpha = 1.0;
    else
        alpha *= 0.90;
    if (alpha < 0.1)
        alpha = 0.1;
}


void world::printSimData() {
    printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f | k: %.3e | fric: %.1f\n",
           currentTime, iter, 0, 0.0, 0.0,
//           m_collisionDetector->num_collisions,
//           m_collisionDetector->min_dist,
//           m_contactPotentialIMC->contact_stiffness,
           mu);
}


void world::newtonMethod(bool &solved) {
    double normf = forceTol * 10.0;
    double normf0 = 0;
    iter = 0;

    double curr_weight = 1.0;
    int counter = 0;
    while (true) {
        for (const auto& limb : limbs) limb->updateGuess(curr_weight);
//        rod->updateGuess(curr_weight);
//        if (m_collisionDetector->constructCandidateSet(counter > 10)) break;
        break;
        curr_weight /= 2;
        counter++;
    }

    cout << timeStep << endl;

    while (solved == false) {
        for (const auto& joint : joints) joint->prepareForIteration();
        for (const auto& limb : limbs) limb->prepareForIteration();
//        rod->prepareForIteration();

        stepper->setZero();

        // Compute the forces and the jacobians
        m_inertialForce->computeFi();
        m_inertialForce->computeJi();

//        normf = 0;
//        for (int i = 0; i < stepper->freeDOF; i++) {
//            normf += totalForce[i] * totalForce[i];
//        }
//        normf = sqrt(normf);
//        cout << "inertial " << normf << endl;

        m_stretchForce->computeFs();
        m_stretchForce->computeJs();

//        normf = 0;
//        for (int i = 0; i < stepper->freeDOF; i++) {
//            normf += totalForce[i] * totalForce[i];
//        }
//        normf = sqrt(normf);
//        cout << "stretching " << normf << endl;

        m_bendingForce->computeFb();
        m_bendingForce->computeJb();

//        normf = 0;
//        for (int i = 0; i < stepper->freeDOF; i++) {
//            normf += totalForce[i] * totalForce[i];
//        }
//        normf = sqrt(normf);
//        cout << "bending " << normf << endl;

//        m_twistingForce->computeFt();
//        m_twistingForce->computeJt();
        m_gravityForce->computeFg();
        m_gravityForce->computeJg();

//        normf = 0;
//        for (int i = 0; i < stepper->freeDOF; i++) {
//            normf += totalForce[i] * totalForce[i];
//        }
//        normf = sqrt(normf);
//        cout << "gravity " << normf << endl;

        m_dampingForce->computeFd();
        m_dampingForce->computeJd();

//        normf = 0;
//        for (int i = 0; i < stepper->freeDOF; i++) {
//            normf += totalForce[i] * totalForce[i];
//        }
//        normf = sqrt(normf);
//        cout << "damping " << normf << endl;

//        m_collisionDetector->detectCollisions();
//        if (iter == 0) {
//            m_contactPotentialIMC->updateContactStiffness();
//        }

//        m_contactPotentialIMC->computeFcJc();

        // Compute norm of the force equations.
        normf = 0;
        for (int i = 0; i < stepper->freeDOF; i++) {
            normf += totalForce[i] * totalForce[i];
        }
        normf = sqrt(normf);

//        cout << stepper->freeDOF << endl;
//        cout << Map<Matrix<double, 4, 4>>(stepper->Force.data()) << endl;
//        cout << "============" << endl;


        if (iter == 0) {
            normf0 = normf;
        }

        if (normf <= forceTol || (iter > 0 && normf <= normf0 * stol)) {
            solved = true;
            iter++;
            if (pulling())
                total_iters++;
        }

        if (solved == false) {
            stepper->integrator(); // Solve equations of motion
//            if (line_search) {
//                lineSearch();
//            } else {
//                newtonDamper();
//            }

            int limb_idx = 0;
            for (const auto& limb : limbs) {
                limb->updateNewtonX(dx, stepper->offsets[limb_idx], alpha);
                limb_idx++;
            }
//            rod->updateNewtonX(dx, alpha); // new q = old q + Delta q
            iter++;
            if (pulling())
                total_iters++;
        }

        // Exit if unable to converge
        if (iter > maxIter) {
            cout << "No convergence after " << maxIter << " iterations" << endl;
            exit(1);
        }
    }
}

int world::simulationRunning() {
    if (currentTime < totalTime)
        return 1;
    else {
        cout << "Completed simulation." << endl;
        return -1;
    }
}

//int world::numPoints() {
//    return rod->nv;
//}

double world::getScaledCoordinate(int i, int limb_idx) {
    return limbs[limb_idx]->x[i] / 0.2;
//    return limbs[limb_idx]->x[i] / (0.20 * RodLength);
//    return rod->x[i] / (0.20 * RodLength);
}

double world::getCurrentTime() {
    return currentTime;
}

void world::lineSearch() {
    // store current x
    for (auto& limb : limbs) {
        limb->xold = limb->x;
    }
//    rod->xold = rod->x;
    // Initialize an interval for optimal learning rate alpha
    double amax = 2;
    double amin = 1e-3;
    double al = 0;
    double au = 1;

    double a = 1;

    //compute the slope initially
    double q0 = 0.5 * pow(stepper->Force.norm(), 2);
    double dq0 = -(stepper->Force.transpose() * stepper->Jacobian * stepper->DX)(0);

    bool success = false;
    double m2 = 0.9;
    double m1 = 0.1;
    int iter_l = 0;
    while (!success) {
        int limb_idx = 0;
        for (auto& limb : limbs) {
            limb->x = limb->xold;
            limb->updateNewtonX(dx, stepper->offsets[limb_idx], alpha);
            limb_idx++;
        }
//        rod->x = rod->xold;
//        rod->updateNewtonX(dx, a);

        for (const auto& joint : joints) joint->prepareForIteration();
        for (const auto& limb : limbs) limb->prepareForIteration();
//        rod->prepareForIteration();

        stepper->setZero();

        // Compute the forces and the jacobians
        m_inertialForce->computeFi();
        m_stretchForce->computeFs();
        m_bendingForce->computeFb();
//        m_twistingForce->computeFt();
        m_gravityForce->computeFg();
        m_dampingForce->computeFd();
//        m_collisionDetector->detectCollisions();
//        m_contactPotentialIMC->computeFc();

        double q = 0.5 * pow(stepper->Force.norm(), 2);

        double slope = (q - q0) / a;

        if (slope >= m2 * dq0 && slope <= m1 * dq0) {
            success = true;
        }
        else {
            if (slope < m2 * dq0) {
                al = a;
            }
            else {
                au = a;
            }

            if (au < amax) {
                a = 0.5 * (al + au);
            }
            else {
                a = 10 * a;
            }
        }
        if (a > amax || a < amin) {
            break;
        }
        if (iter_l > 100) {
            break;
        }
        iter_l++;
    }
    for (auto& limb : limbs) {
        limb->x = limb->xold;
    }
//    rod->x = rod->xold;

    alpha = a;
}
