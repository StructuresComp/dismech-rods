#ifndef IMPLICITMIDPOINT_H
#define IMPLICITMIDPOINT_H

#include "backwardEuler.h"

class implicitMidpoint : public backwardEuler
{
public:
    implicitMidpoint(const shared_ptr<softRobots>& soft_robots,
                     const shared_ptr<forceContainer>& forces,
                     const simParams& sim_params, solverType solver_type);
    ~implicitMidpoint() override;

    double stepForwardInTime() override;
};

#endif
