#include "dampingForce.h"
#include <iostream>

dampingForce::dampingForce(elasticRod &m_rod, timeStepper &m_stepper, double m_viscosity)
{
    rod = &m_rod;
    stepper = &m_stepper;
    viscosity = m_viscosity;
}

dampingForce::~dampingForce()
{
    ;
}

void dampingForce::computeFd()
{
    for (int i = 0; i < rod->ne; i++)
    {
        force = -viscosity * (rod->getVertex(i) - rod->getPreVertex(i))  / rod->dt * rod->voronoiLen(i);
        for (int k = 0; k < 3; k++)
        {
            ind = 4 * i + k;
            stepper->addForce(ind, -force[k]); // subtracting external force
        }
    }
}

void dampingForce::computeJd()
{
    // Here, we take advantage of the fact that the damping force Jacobian is a diagonal matrix of identical values.
    for (int i = 0; i < rod->ne; i++)
    {
        jac = -viscosity * rod->voronoiLen(i) / rod->dt;
        for (int k = 0; k < 3; k++)
        {
            ind = 4 * i + k;
            stepper->addJacobian(ind, ind, -jac);
        }
    }
}
