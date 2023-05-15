#include "world.h"

world::world()
{
    ;
}

world::world(setInput &m_inputData)
{
    render = m_inputData.GetBoolOpt("render");                         // boolean
    gVector = m_inputData.GetVecOpt("gVector");                        // m/s^2
    maxIter = m_inputData.GetIntOpt("maxIter");                        // maximum number of iterations
    rodRadius = m_inputData.GetScalarOpt("rodRadius");                 // meter
    youngM = m_inputData.GetScalarOpt("youngM");                       // Pa
    Poisson = m_inputData.GetScalarOpt("Poisson");                     // dimensionless
    deltaTime = m_inputData.GetScalarOpt("deltaTime");                 // seconds
    tol = m_inputData.GetScalarOpt("tol");                             // small number like 10e-7
    stol = m_inputData.GetScalarOpt("stol");                           // small number, e.g. 0.1%
    density = m_inputData.GetScalarOpt("density");                     // kg/m^3
    viscosity = m_inputData.GetScalarOpt("viscosity");                 // viscosity in Pa-s
    data_resolution = m_inputData.GetScalarOpt("dataResolution");      // time resolution for recording data
    col_limit = m_inputData.GetScalarOpt("colLimit");                  // distance limit for candidate set
    delta = m_inputData.GetScalarOpt("delta");                         // distance tolerance for contact
    k_scaler = m_inputData.GetScalarOpt("kScaler");                    // constant scaler for contact stiffness
    mu = m_inputData.GetScalarOpt("mu");                               // friction coefficient
    nu = m_inputData.GetScalarOpt("nu");                               // slipping tolerance for friction
    line_search = m_inputData.GetIntOpt("lineSearch");                 // flag for enabling line search
    floor_z = m_inputData.GetScalarOpt("floorZ");                      // z-coordinate of floor plane
    totalTime = m_inputData.GetScalarOpt("simTime");                   // simulation duration
    phi_ctrl_filepath = m_inputData.GetStringOpt("phiCtrlFilePath"); // controller setpoints (bending angle phi for limbs)

    shearM = youngM / (2.0 * (1.0 + Poisson)); // shear modulus

    data_rate = ceil(data_resolution / deltaTime); // iter resolution for recording data
    alpha = 1.0;                                   // newton step size
    total_iters = 0;                               // total number of newton iterations
}

world::~world()
{
    ;
}

bool world::isRender()
{
    return render;
}

void world::OpenFile(ofstream &outfile, string file_type)
{
    int systemRet = system("mkdir datafiles"); // make the directory
    if (systemRet == -1)
    {
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

// TODO: remove this in favor of the logging classes later
void world::outputNodeCoordinates(ofstream &outfile)
{
    if (timeStep % data_rate != 0)
        return;
    Vector3d curr_node;
    double curr_theta;
    for (int i = 0; i < rod->nv - 1; i++)
    {
        curr_node = rod->getVertex(i);
        curr_theta = rod->getTheta(i);
        outfile << curr_node(0) << " " << curr_node(1) << " " << curr_node(2) << " " << curr_theta << endl;
    }
    curr_node = rod->getVertex(rod->nv - 1);
    outfile << curr_node(0) << " " << curr_node(1) << " " << curr_node(2) << " " << 0.0 << endl;
}

void world::CloseFile(ofstream &outfile)
{
    outfile.close();
}

void world::setupWorld()
{
    get_robot_description(limbs, joints, density, rodRadius, deltaTime, youngM, shearM);
    setupController(controllers, limbs, phi_ctrl_filepath);
    // This has to be called after joints are all set.
    for (const auto &joint : joints)
        joint->setup();

    // TODO: make characteristicForce a function of total cumulative rod length?
    double temp_value = 1.0;
    // Find out the tolerance, e.g. how small is enough?
    characteristicForce = M_PI * pow(rodRadius, 4) / 4.0 * youngM / pow(temp_value, 2);
    forceTol = tol * characteristicForce;

    // set up the time stepper
    stepper = make_shared<timeStepper>(limbs);
    totalForce = stepper->getForce();
    dx = stepper->dx;

    // Set up boundary condition
    //    lockEdge(0, 0);
    //    lockEdge(0, 1);
    updateCons();

    // declare the forces
    m_stretchForce = make_unique<elasticStretchingForce>(limbs, joints, stepper);
    m_bendingForce = make_unique<elasticBendingForce>(limbs, joints, stepper);
    m_twistingForce = make_unique<elasticTwistingForce>(limbs, joints, stepper);
    m_inertialForce = make_unique<inertialForce>(limbs, joints, stepper);
    m_gravityForce = make_unique<externalGravityForce>(limbs, joints, stepper, gVector);
    m_dampingForce = make_unique<dampingForce>(limbs, joints, stepper, viscosity);
    m_floorContactForce = make_unique<floorContactForce>(limbs, stepper, delta, nu, mu, deltaTime, floor_z);

    //    m_collisionDetector = make_shared<collisionDetector>(rod, delta, col_limit);
    //    m_contactPotentialIMC = make_unique<contactPotentialIMC>(rod, stepper, m_collisionDetector, delta, k_scaler, mu, nu);

    // Allocate every thing to prepare for the first iteration
    updateRobot();

    currentTime = 0.0;
    timeStep = 0;
}

void world::setupController(vector<shared_ptr<rodOpenLoopFileKappabarSetter>> &controllers, vector<shared_ptr<elasticRod>> &limbs, string phi_ctrl_filepath)
{
    int num_limb;
    num_limb = limbs.size();
    cout << "start" << endl;
    cout << phi_ctrl_filepath << endl;
    cout << "start1" << endl;
    controllers.push_back(make_shared<rodOpenLoopFileKappabarSetter>(num_limb, phi_ctrl_filepath, limbs));
}

void world::updateRobot()
{
    for (const auto &joint : joints)
        joint->prepLimbs();
    for (const auto &controller : controllers)
        controller->updateTimestep(deltaTime);
    for (const auto &limb : limbs)
        limb->updateTimeStep();
    for (const auto &joint : joints)
        joint->updateTimeStep();
}

void world::prepRobot()
{
    for (const auto &joint : joints)
        joint->prepLimbs();
    for (const auto &limb : limbs)
        limb->prepareForIteration();
    for (const auto &joint : joints)
        joint->prepareForIteration();
}

// TODO: this is hardcoded, fix later when needed
void world::lockEdge(int edge_num, int limb_idx)
{
    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(0), 0);
    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(1), 1);
    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 0);
    //    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 1);

    //    limbs[limb_idx]->setThetaBoundaryCondition(0.0, 1);
    //    limbs[limb_idx]->setVertexBoundaryCondition(limbs[limb_idx]->getVertex(2), 2);
}

void world::updateCons()
{
    for (const auto &limb : limbs)
        limb->updateMap();
    stepper->update();
    totalForce = stepper->getForce();
    dx = stepper->dx;
}

int world::getTimeStep()
{
    return timeStep;
}

void world::updateTimeStep()
{
    bool solved = false;

    newtonMethod(solved);

    updateRobot();

    printSimData();

    currentTime += deltaTime;
    timeStep++;
}

void world::printSimData()
{
    printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f | k: %.3e | fric: %.1f\n",
           currentTime, iter, 0,
           m_floorContactForce->min_dist,
           0.0,
           //           m_collisionDetector->num_collisions,
           //           m_collisionDetector->min_dist,
           //           m_contactPotentialIMC->contact_stiffness,
           mu);
}

void world::newtonMethod(bool &solved)
{
    double normf = forceTol * 10.0;
    double normf0 = 0;
    iter = 0;

    for (const auto &limb : limbs)
        limb->updateGuess(0.01);

    while (solved == false)
    {
        prepRobot();

        stepper->setZero();

        // Compute the forces and the jacobians
        m_inertialForce->computeFi();
        m_inertialForce->computeJi();

        m_stretchForce->computeFs();
        m_stretchForce->computeJs();

        m_bendingForce->computeFb();
        m_bendingForce->computeJb();

        m_twistingForce->computeFt();
        m_twistingForce->computeJt();

        m_gravityForce->computeFg();
        m_gravityForce->computeJg();

        m_dampingForce->computeFd();
        m_dampingForce->computeJd();

        m_floorContactForce->computeFfJf();

        //        m_collisionDetector->detectCollisions();
        //        if (iter == 0) {
        //            m_contactPotentialIMC->updateContactStiffness();
        //        }

        //        m_contactPotentialIMC->computeFcJc();

        // Compute norm of the force equations.
        normf = 0;
        for (int i = 0; i < stepper->freeDOF; i++)
        {
            normf += totalForce[i] * totalForce[i];
        }
        normf = sqrt(normf);

        if (iter == 0)
        {
            normf0 = normf;
        }

        if (normf <= forceTol || (iter > 0 && normf <= normf0 * stol))
        {
            solved = true;
            iter++;
        }

        if (solved == false)
        {
            stepper->integrator(); // Solve equations of motion
            if (line_search)
                lineSearch();

            int limb_idx = 0;
            for (const auto &limb : limbs)
            {
                limb->updateNewtonX(dx, stepper->offsets[limb_idx], alpha);
                limb_idx++;
            }
            iter++;
        }

        // Exit if unable to converge
        if (iter > maxIter)
        {
            cout << "No convergence after " << maxIter << " iterations" << endl;
            exit(1);
        }
    }
}

int world::simulationRunning()
{
    if (currentTime < totalTime)
        return 1;
    else
    {
        cout << "Completed simulation." << endl;
        return -1;
    }
}

double world::getScaledCoordinate(int i, int limb_idx)
{
    return limbs[limb_idx]->x[i] / 0.2;
    //    return limbs[limb_idx]->x[i] / (0.20 * RodLength);
    //    return rod->x[i] / (0.20 * RodLength);
}

double world::getCurrentTime()
{
    return currentTime;
}

void world::lineSearch()
{
    // store current x
    for (auto &limb : limbs)
    {
        limb->xold = limb->x;
    }
    for (auto &joint : joints)
    {
        joint->xold = joint->x;
    }
    // Initialize an interval for optimal learning rate alpha
    double amax = 2;
    double amin = 1e-3;
    double al = 0;
    double au = 1;

    double a = 1;

    // compute the slope initially
    double q0 = 0.5 * pow(stepper->Force.norm(), 2);
    double dq0 = -(stepper->Force.transpose() * stepper->Jacobian * stepper->DX)(0);

    bool success = false;
    double m2 = 0.9;
    double m1 = 0.1;
    int iter_l = 0;
    while (!success)
    {
        int limb_idx = 0;
        for (auto &joint : joints)
        {
            joint->x = joint->xold;
        }
        for (auto &limb : limbs)
        {
            limb->x = limb->xold;
            limb->updateNewtonX(dx, stepper->offsets[limb_idx], alpha);
            limb_idx++;
        }

        prepRobot();

        stepper->setZero();

        // Compute the forces and the jacobians
        m_inertialForce->computeFi();
        m_stretchForce->computeFs();
        m_bendingForce->computeFb();
        m_twistingForce->computeFt();
        m_gravityForce->computeFg();
        m_dampingForce->computeFd();
        m_floorContactForce->computeFf();
        //        m_collisionDetector->detectCollisions();
        //        m_contactPotentialIMC->computeFc();

        double q = 0.5 * pow(stepper->Force.norm(), 2);

        double slope = (q - q0) / a;

        if (slope >= m2 * dq0 && slope <= m1 * dq0)
        {
            success = true;
        }
        else
        {
            if (slope < m2 * dq0)
            {
                al = a;
            }
            else
            {
                au = a;
            }

            if (au < amax)
            {
                a = 0.5 * (al + au);
            }
            //            else {
            //                a = 10 * a;
            //            }
        }
        if (a > amax || a < amin)
        {
            break;
        }
        if (iter_l > 100)
        {
            break;
        }
        iter_l++;
    }
    for (auto &limb : limbs)
    {
        limb->x = limb->xold;
    }
    for (auto &joint : joints)
    {
        joint->x = joint->xold;
    }
    alpha = a;
}
