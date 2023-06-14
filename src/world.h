#ifndef WORLD_H
#define WORLD_H

#include "eigenIncludes.h"
#include "robotDescription.h"

// include elastic rod class
#include "rod_mechanics/elasticRod.h"
#include "rod_mechanics/elasticJoint.h"
#include "controllers/rodController.h"
#include "controllers/rodOpenLoopFileKappabarSetter.h"
// include force classes
#include "rod_mechanics/elasticStretchingForce.h"
#include "rod_mechanics/elasticBendingForce.h"
#include "rod_mechanics/elasticTwistingForce.h"
#include "rod_mechanics/inertialForce.h"

// include external force
#include "rod_mechanics/dampingForce.h"
#include "rod_mechanics/externalGravityForce.h"
#include "rod_mechanics/floorContactForce.h"
// #include "rod_mechanics/contactPotentialIMC.h"

// include time stepper
//#include "time_steppers/timeStepper.h"
#include "time_steppers/verletPosition.h"
#include "time_steppers/backwardEuler.h"
#include "time_steppers/implicitMidpoint.h"

// include input file and option
#include "initialization/setInput.h"

class world
{
public:
    world();
    world(setInput &m_inputData);
    ~world();
    void setupWorld();
    void updateTimeStep();
    int simulationRunning();
    double getScaledCoordinate(int i, int limb_idx);
    VectorXd getM1(int i, int limb_idx);
    VectorXd getM2(int i, int limb_idx);
    double getCurrentTime();

    bool isRender();

    // file output
    void OpenFile(ofstream &outfile, string filename);
    void CloseFile(ofstream &outfile);
    void outputNodeCoordinates(ofstream &outfile);

    int getTimeStep();

    // TODO: Create more sophisticated classes for these
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<elasticJoint>> joints;
    vector<shared_ptr<rodController>> controllers;

private:
    // Physical parameters
    double rodRadius;
    double youngM;
    double Poisson;
    double shearM;
    double deltaTime;
    double density;
    Vector3d gVector;
    double viscosity;
    double col_limit;
    double delta;
    double k_scaler;
    double mu;
    double nu;
    double data_resolution;
    int data_rate;
    bool line_search;
    string knot_config;
    double alpha;
    double floor_z;

    double tol, stol;
    int maxIter; // maximum number of iterations
    double characteristicForce;
    double forceTol;
    string integration_scheme;
    bool enable_2d_sim;

    // Geometry
    MatrixXd vertices;
    VectorXd theta;

    // set up the time stepper
    shared_ptr<baseTimeStepper> stepper = nullptr;
    double *totalForce;
    double *dx;

    double currentTime;
    int timeStep;
    double totalTime;

    // declare the forces
    shared_ptr<elasticStretchingForce> m_stretchForce = nullptr;
    shared_ptr<elasticBendingForce> m_bendingForce = nullptr;
    shared_ptr<elasticTwistingForce> m_twistingForce = nullptr;
    shared_ptr<inertialForce> m_inertialForce = nullptr;
    shared_ptr<externalGravityForce> m_gravityForce = nullptr;
    shared_ptr<dampingForce> m_dampingForce = nullptr;
    shared_ptr<floorContactForce> m_floorContactForce = nullptr;
//    shared_ptr<collisionDetector> m_collisionDetector = nullptr;
//    shared_ptr<contactPotentialIMC> m_contactPotentialIMC = nullptr;

    // controller parameter and setup
    string phi_ctrl_filepath;
    void setupController(vector<shared_ptr<rodController>> &controllers, vector<shared_ptr<elasticRod>> &limbs, string phi_ctrl_filepath);

    void updateCons();

    void printSimData();

    bool render; // should the OpenGL rendering be included?

    Vector3d temp;
    Vector3d temp1;
    Vector3d gravity;
    Vector3d inertial;
    Vector3d dampingF;
};

#endif
