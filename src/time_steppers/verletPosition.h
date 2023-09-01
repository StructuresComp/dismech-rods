#ifndef VERLETPOSITION_H
#define VERLETPOSITION_H

#include "explicitTimeStepper.h"

class verletPosition : public explicitTimeStepper
{
public:
    verletPosition(const shared_ptr<softRobots>& m_soft_robots,
                   const shared_ptr<innerForces>& m_inner_forces,
                   const shared_ptr<externalForces>& m_external_forces,
                   double m_dt);
    ~verletPosition() override;

    double stepForwardInTime() override;

    void updateSystemForNextTimeStep() override;

    void constructInverseMassVector();

    vector<VectorXd> inverse_masses;
};


#endif
