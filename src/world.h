#ifndef WORLD_H
#define WORLD_H

#include "global_definitions.h"

class SoftRobots;
class ForceContainer;
class BaseTimeStepper;

class World
{
  public:
    World(const std::shared_ptr<SoftRobots>& soft_robots,
          const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params);
    ~World();
    void updateTimeStep();
    double getCoordinate(int i, int limb_idx);
    VecX getM1(int i, int limb_idx);
    VecX getM2(int i, int limb_idx);

    double getCurrentTime() const;
    bool simulationRunning() const;
    int getTimeStep() const;
    void printSimData();
    bool floorExists();
    double getFloorZ();

    std::shared_ptr<SoftRobots> soft_robots;

  private:
    std::shared_ptr<ForceContainer> forces;
    std::shared_ptr<BaseTimeStepper> stepper;

    int time_step;
    double curr_time;
    double total_time;

    void updateCons();
};

#endif
