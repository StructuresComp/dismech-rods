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
//#include "rod_mechanics/contactPotentialIMC.h"

// include time stepper
#include "rod_mechanics/timeStepper.h"

// include input file and option
#include "initialization/setInput.h"

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
    double getScaledCoordinate(int i, int limb_idx);
    double getCurrentTime();

    bool pulling();
    bool isRender();

    // file output
    void OpenFile(ofstream &outfile, string filename);
    void CloseFile(ofstream &outfile);
    void outputNodeCoordinates(ofstream &outfile);
    bool CoutDataC(ofstream &outfile);

    void updateTimeStep_data();

    int getTimeStep();

    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<Joint>> joints;

    void makeJoint(int node, int limb_idx);

    void addToJoint(int joint_num, int node, int limb_idx);


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
    shared_ptr<elasticRod> rod = nullptr;

    // set up the time stepper
    shared_ptr<timeStepper> stepper = nullptr;
    double *totalForce;
    double *dx;

    double currentTime;
    int timeStep;
    double totalTime;

    // declare the forces
    unique_ptr<elasticStretchingForce> m_stretchForce = nullptr;
    unique_ptr<elasticBendingForce> m_bendingForce = nullptr;
    unique_ptr<elasticTwistingForce> m_twistingForce = nullptr;
    unique_ptr<inertialForce> m_inertialForce = nullptr;
    unique_ptr<externalGravityForce> m_gravityForce = nullptr;
    unique_ptr<dampingForce> m_dampingForce = nullptr;
//    shared_ptr<collisionDetector> m_collisionDetector = nullptr;
//    unique_ptr<contactPotentialIMC> m_contactPotentialIMC = nullptr;

    int iter;
    int total_iters;

    void rodGeometry();
    void rodBoundaryCondition();
    void lockEdge(int edge_num, int limb_idx);

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
