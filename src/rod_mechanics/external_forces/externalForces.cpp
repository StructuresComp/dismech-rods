#include "externalForces.h"


externalForces::externalForces(shared_ptr<externalGravityForce> m_gravity_force,
                               shared_ptr<dampingForce> m_damping_force,
                               shared_ptr<floorContactForce> m_floor_contact_force,
                               shared_ptr<contactForce> m_contact_force) :
                               baseForceContainer(),
                               gravity_force(std::move(m_gravity_force)),
                               damping_force(std::move(m_damping_force)),
                               floor_contact_force(std::move(m_floor_contact_force)),
                               contact_force(std::move(m_contact_force))
{
    if (gravity_force)
        forces.push_back(gravity_force);
    if (damping_force)
        forces.push_back(damping_force);
    if (floor_contact_force)
        forces.push_back(floor_contact_force);
    if (contact_force)
        forces.push_back(contact_force);
}


externalForces::~externalForces() = default;


void externalForces::addToForces(const vector<shared_ptr<baseForce>> &forces_to_add) {
    for (const auto& force_to_add : forces_to_add)
        forces.emplace_back(force_to_add);
}
