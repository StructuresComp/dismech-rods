#ifndef BACKWARDEULER_H
#define BACKWARDEULER_H

#include "implicitTimeStepper.h"

class backwardEuler : public implicitTimeStepper
{
public:
    backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs,
                  shared_ptr<elasticStretchingForce> m_stretchForce,
                  shared_ptr<elasticBendingForce> m_bendingForce,
                  shared_ptr<elasticTwistingForce> m_twistingForce,
                  shared_ptr<inertialForce> m_inertialForce,
                  shared_ptr<externalGravityForce> m_gravityForce,
                  shared_ptr<dampingForce> m_dampingForce,
                  shared_ptr<floorContactForce> m_floorContactForce);
    ~backwardEuler() override;

    void integrator() override;

};


#endif
