#ifndef ACTIVE_ENTANGLEMENT_CONTROLLER_H
#define ACTIVE_ENTANGLEMENT_CONTROLLER_H

#include "base_controller.h"

class SoftRobots;

class ActiveEntanglementController : public BaseController
{
  public:
    explicit ActiveEntanglementController(const std::shared_ptr<SoftRobots>& soft_robots,
                                          double start_time, double end_time);
    ~ActiveEntanglementController();

    void updateTimeStep(double dt) override;

  private:
    std::vector<std::vector<double>> random_curvatures;
    double start_time;
    double end_time;
};

#endif  // ACTIVE_ENTANGLEMENT_CONTROLLER_H
