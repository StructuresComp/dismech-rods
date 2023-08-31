#include "externalForces.h"


externalForces::externalForces(shared_ptr<externalGravityForce> m_gravity_force,
                               shared_ptr<dampingForce> m_damping_force,
                               shared_ptr<floorContactForce> m_floor_contact_force) :
                               baseForceContainer(),
                               gravity_force(std::move(m_gravity_force)), damping_force(std::move(m_damping_force)),
                               floor_contact_force(std::move(m_floor_contact_force))
{
    if (gravity_force)
        forces.push_back(gravity_force);
    if (damping_force)
        forces.push_back(damping_force);
    if (floor_contact_force)
        forces.push_back(floor_contact_force);
}


externalForces::~externalForces() = default;
