#ifndef EXPLICITTIMESTEPPER_H
#define EXPLICITTIMESTEPPER_H

#include "baseTimeStepper.h"

class explicitTimeStepper : public baseTimeStepper
{
public:
    explicitTimeStepper(const vector<shared_ptr<elasticRod>>& m_limbs,
            // These are simply overwritten to
                        const vector<shared_ptr<elasticJoint>>& m_joints,
                        shared_ptr<elasticStretchingForce> m_stretch_force,
                        shared_ptr<elasticBendingForce> m_bending_force,
                        shared_ptr<elasticTwistingForce> m_twisting_force,
                        shared_ptr<inertialForce> m_inertial_force,
                        shared_ptr<externalGravityForce> m_gravity_force,
                        shared_ptr<dampingForce> m_damping_force,
                        shared_ptr<floorContactForce> m_floor_contact_force,
                        double m_dt);
    ~explicitTimeStepper() override;

    void prepSystemForIteration() override;
    double* getJacobian() override;
    void integrator() override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_indx1, int limb_idx2) override;

};


#endif
