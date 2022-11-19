#include "externalGravityForce.h"

externalGravityForce::externalGravityForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, Vector3d m_gVector)
{
    rod = m_rod;
    stepper = m_stepper;
    gVector = m_gVector;
    setGravity();
}

externalGravityForce::~externalGravityForce()
{
    ;
}

void externalGravityForce::computeFg()
{
    for (int i=0; i < rod->ndof; i++)
    {
        stepper->addForce(i, -massGravity[i]); // subtracting gravity force
    }
}

void externalGravityForce::computeJg()
{
    ;
}

void externalGravityForce::setGravity()
{
    massGravity = VectorXd::Zero(rod->ndof);
    for (int i=0; i < rod->nv; i++)
    {
        for (int k=0; k < 3; k++)
        {
            int ind = 4*i + k;
            massGravity[ind] = gVector[k] * rod->massArray[ind];
        }
    }
}
