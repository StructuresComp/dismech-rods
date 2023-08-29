#ifndef VERLETPOSITION_H
#define VERLETPOSITION_H

#include "explicitTimeStepper.h"

class verletPosition : public explicitTimeStepper
{
public:
    verletPosition(const vector<shared_ptr<elasticRod>>& m_limbs,
                   const vector<shared_ptr<elasticJoint>>& m_joints,
                   const vector<shared_ptr<rodController>>& m_controllers,
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
