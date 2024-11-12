#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "global_definitions.h"

class ElasticRod;

class BaseController
{
  public:
    explicit BaseController(const std::vector<std::shared_ptr<ElasticRod>>& limbs);
    ~BaseController();

    virtual void updateTimeStep(double dt);

  protected:
    std::vector<std::shared_ptr<ElasticRod>> limbs;
    int num_actuators;
    double current_time;
};

#endif  // BASE_CONTROLLER_H
