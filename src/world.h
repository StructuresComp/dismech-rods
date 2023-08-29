#ifndef WORLD_H
#define WORLD_H

#include "eigenIncludes.h"
#include "robotDescription.h"

// include elastic rod class
#include "rod_mechanics/elasticRod.h"
#include "rod_mechanics/elasticJoint.h"

// include controllers
#include "controllers/rodController.h"
#include "controllers/rodOpenLoopFileKappabarSetter.h"

// include inner force classes
#include "rod_mechanics/inner_forces/inertialForce.h"
#include "rod_mechanics/inner_forces/elasticStretchingForce.h"
#include "rod_mechanics/inner_forces/elasticBendingForce.h"
#include "rod_mechanics/inner_forces/elasticTwistingForce.h"

// include external forces
#include "rod_mechanics/external_forces/dampingForce.h"
#include "rod_mechanics/external_forces/externalGravityForce.h"
#include "rod_mechanics/external_forces/floorContactForce.h"
// #include "rod_mechanics/external_forces/contactPotentialIMC.h"

// include time stepper
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
    void printSimData();

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
    int adaptive_time_stepping;

    // Geometry
    MatrixXd vertices;
    VectorXd theta;

    // set up the time stepper
    shared_ptr<baseTimeStepper> stepper = nullptr;

    double currentTime;
    int timeStep;
    double totalTime;

    // declare the forces
    shared_ptr<innerForces> inner_forces;
    shared_ptr<externalForces> external_forces;

    // controller parameter and setup
    string phi_ctrl_filepath;
    void setupController(vector<shared_ptr<rodController>> &controllers, vector<shared_ptr<elasticRod>> &limbs, string phi_ctrl_filepath);

    void updateCons();

    bool render; // should the OpenGL rendering be included?

    Vector3d temp;
    Vector3d temp1;
    Vector3d gravity;
    Vector3d inertial;
    Vector3d dampingF;
};

#endif
