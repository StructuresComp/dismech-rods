#ifndef WORLD_H
#define WORLD_H

#include "global_definitions.h"

#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

// include inner force classes
#include "rod_mechanics/inner_forces/elastic_bending_force.h"
#include "rod_mechanics/inner_forces/elastic_stretching_force.h"
#include "rod_mechanics/inner_forces/elastic_twisting_force.h"
#include "rod_mechanics/inner_forces/inertial_force.h"

// include time stepper
#include "time_steppers/backward_euler.h"
#include "time_steppers/forward_euler.h"
#include "time_steppers/implicit_midpoint.h"
#include "time_steppers/verlet_position.h"

class World
{
  public:
    World(const shared_ptr<SoftRobots>& soft_robots, const shared_ptr<ForceContainer>& forces,
          const SimParams& sim_params);
    ~World();
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

    shared_ptr<SoftRobots> soft_robots;

  private:
    shared_ptr<ForceContainer> forces;
    shared_ptr<BaseTimeStepper> stepper;

    int time_step;
    double curr_time;
    double total_time;

    void updateCons();
};

#endif
