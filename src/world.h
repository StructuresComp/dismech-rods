#ifndef WORLD_H
#define WORLD_H

#include "globalDefinitions.h"

#include "rod_mechanics/forceContainer.h"
#include "rod_mechanics/softRobots.h"

// include inner force classes
#include "rod_mechanics/inner_forces/elasticBendingForce.h"
#include "rod_mechanics/inner_forces/elasticStretchingForce.h"
#include "rod_mechanics/inner_forces/elasticTwistingForce.h"
#include "rod_mechanics/inner_forces/inertialForce.h"

// include time stepper
#include "time_steppers/backwardEuler.h"
#include "time_steppers/forwardEuler.h"
#include "time_steppers/implicitMidpoint.h"
#include "time_steppers/verletPosition.h"

class world
{
  public:
    world(const shared_ptr<softRobots>& soft_robots, const shared_ptr<forceContainer>& forces,
          const simParams& sim_params);
    ~world();
    void updateTimeStep();
    double getCoordinate(int i, int limb_idx);
    VectorXd getM1(int i, int limb_idx);
    VectorXd getM2(int i, int limb_idx);

    double getCurrentTime() const;
    bool simulationRunning() const;
    int getTimeStep() const;
    void printSimData();
    bool floorExists();
    double getFloorZ();

    shared_ptr<softRobots> soft_robots;

  private:
    shared_ptr<forceContainer> forces;
    shared_ptr<baseTimeStepper> stepper;

    int time_step;
    double curr_time;
    double total_time;

    void updateCons();
};

#endif
