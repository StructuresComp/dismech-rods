#ifndef ACTIVE_ENTANGLEMENT_CONTROLLER_H
#define ACTIVE_ENTANGLEMENT_CONTROLLER_H

#include "baseController.h"

class softRobots;

class activeEntanglementController : public baseController
{
public:
    explicit activeEntanglementController(const shared_ptr<softRobots>& soft_robots, double start_time, double end_time);
    ~activeEntanglementController();

    void updateTimeStep(double dt) override;

private:
    vector<vector<double>> random_curvatures;
    double start_time;
    double end_time;
};

#endif
