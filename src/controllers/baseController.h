#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "eigenIncludes.h"

class elasticRod;

class baseController
{
public:
    explicit baseController(const vector<shared_ptr<elasticRod>>& limbs);
    ~baseController();

    virtual void updateTimeStep(double dt);

protected:
    vector<shared_ptr<elasticRod>> limbs;
    int num_actuators;
    double current_time;
};

#endif