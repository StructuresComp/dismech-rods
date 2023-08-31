#ifndef INNER_FORCES_H
#define INNER_FORCES_H

#include "eigenIncludes.h"
#include "rod_mechanics/baseForceContainer.h"
#include "inertialForce.h"
#include "elasticStretchingForce.h"
#include "elasticBendingForce.h"
#include "elasticTwistingForce.h"


class innerForces : public baseForceContainer
{
public:
    innerForces(shared_ptr<inertialForce> m_inertial_force, shared_ptr<elasticStretchingForce> m_stretching_force,
                shared_ptr<elasticBendingForce> m_bending_force, shared_ptr<elasticTwistingForce> m_twisting_force);
    ~innerForces();

private:
    shared_ptr<inertialForce> inertial_force = nullptr;
    shared_ptr<elasticStretchingForce> stretching_force = nullptr;
    shared_ptr<elasticBendingForce> bending_force = nullptr;
    shared_ptr<elasticTwistingForce> twisting_force = nullptr;
};


#endif