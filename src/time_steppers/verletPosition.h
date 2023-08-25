#ifndef VERLETPOSITION_H
#define VERLETPOSITION_H

#include "explicitTimeStepper.h"

class verletPosition : public explicitTimeStepper
{
public:
    verletPosition(const vector<shared_ptr<elasticRod>>& m_limbs,
                   const vector<shared_ptr<elasticJoint>>& m_joints,
                   const vector<shared_ptr<rodController>>& m_controllers,
                   shared_ptr<elasticStretchingForce> m_stretch_force,
                   shared_ptr<elasticBendingForce> m_bending_force,
                   shared_ptr<elasticTwistingForce> m_twisting_force,
                   shared_ptr<inertialForce> m_inertial_force,
                   shared_ptr<externalGravityForce> m_gravity_force,
                   shared_ptr<dampingForce> m_damping_force,
                   shared_ptr<floorContactForce> m_floor_contact_force,
                   double m_dt);
    ~verletPosition() override;

    double stepForwardInTime() override;

    void updateSystemForNextTimeStep() override;

    void constructInverseMassVector();

    vector<VectorXd> inverse_masses;
};


#endif
