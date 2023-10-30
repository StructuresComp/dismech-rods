#ifndef ROD_CONTROLLER_H
#define ROD_CONTROLLER_H

#include "eigenIncludes.h"

class elasticRod;

class rodController
{
public:
    explicit rodController(const vector<shared_ptr<elasticRod>>& limbs);
    ~rodController();

    virtual void updateTimestep(double dt);

protected:
    vector<shared_ptr<elasticRod>> limbs;
    int num_actuators;
    double current_time;
};

#endif //ROD_CONTROLLER_H