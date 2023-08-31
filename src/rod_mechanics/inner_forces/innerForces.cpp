#include "innerForces.h"

#include <utility>


innerForces::innerForces(shared_ptr<inertialForce> m_inertial_force,
                         shared_ptr<elasticStretchingForce> m_stretching_force,
                         shared_ptr<elasticBendingForce> m_bending_force,
                         shared_ptr<elasticTwistingForce> m_twisting_force) :
                         baseForceContainer(),
                         inertial_force(std::move(m_inertial_force)), stretching_force(std::move(m_stretching_force)),
                         bending_force(std::move(m_bending_force)), twisting_force(std::move(m_twisting_force))
{
    if (inertial_force)
        forces.push_back(inertial_force);
    if (stretching_force)
        forces.push_back(stretching_force);
    if (bending_force)
        forces.push_back(bending_force);
    if (twisting_force)
        forces.push_back(twisting_force);

}


innerForces::~innerForces() = default;
