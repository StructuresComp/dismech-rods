#ifndef WORLD_H
#define WORLD_H

#include "eigenIncludes.h"

// include elastic rod class
#include "rod_mechanics/elasticRod.h"

// include force classes
#include "rod_mechanics/elasticStretchingForce.h"
#include "rod_mechanics/elasticBendingForce.h"
#include "rod_mechanics/elasticTwistingForce.h"
#include "rod_mechanics/inertialForce.h"

// include external force
#include "rod_mechanics/dampingForce.h"
#include "rod_mechanics/externalGravityForce.h"
#include "rod_mechanics/contactPotentialIMC.h"

// include time stepper
#include "rod_mechanics/timeStepper.h"

// include input file and option
#include "initialization/setInput.h"

extern double* meta_data;

class world
{
public:
    world();
    world(setInput &m_inputData);
    ~world();
    void setRodStepper();
    void updateTimeStep();
    int simulationRunning();
    int numPoints();
    double getScaledCoordinate(int i);
    double getCurrentTime();

    bool pulling();
    bool isRender();

    // file output
    void OpenFile(ofstream &outfile, string filename);
    void CloseFile(ofstream &outfile);
    void outputNodeCoordinates(ofstream &outfile);
    bool CoutDataC(ofstream &outfile);

    void updateTimeStep_data();


private:

    // Physical parameters
    double RodLength;
    double helixradius, helixpitch;
    double rodRadius;
    int numVertices;
    double youngM;
    double Poisson;
    double shearM;
    double deltaTime;
    double density;
    Vector3d gVector;
    double viscosity;
    double pull_time;
    double release_time;
    double wait_time;
    double pull_speed;
    double col_limit;
    double delta;
    double k_scaler;
    double mu;
    double nu;
    double data_resolution;
    int data_rate;
    int line_search;
    string knot_config;
    double alpha;

    double tol, stol;
    int maxIter; // maximum number of iterations
    double characteristicForce;
    double forceTol;

    // Geometry
    MatrixXd vertices;
    VectorXd theta;

    // Rod
    elasticRod *rod;

    // set up the time stepper
    timeStepper *stepper;
    double *totalForce;
    double *dx;
    double *ls_nodes;
    double currentTime;
    int timeStep;
    double totalTime;

    // declare the forces
    elasticStretchingForce *m_stretchForce;
    elasticBendingForce *m_bendingForce;
    elasticTwistingForce *m_twistingForce;
    inertialForce *m_inertialForce;
    externalGravityForce *m_gravityForce;
    dampingForce *m_dampingForce;
    collisionDetector *m_collisionDetector;
    contactPotentialIMC *m_contactPotentialIMC;

    int iter;
    int total_iters;

    void rodGeometry();
    void rodBoundaryCondition();

    void updateBoundary();

    void updateCons();

    void newtonMethod(bool &solved);
    void printSimData();
    void newtonDamper();
    void calculateForce();
    void lineSearch();

    bool render; // should the OpenGL rendering be included?

    Vector3d temp;
    Vector3d temp1;
    Vector3d gravity;
    Vector3d inertial;
    Vector3d dampingF;

};

#endif
