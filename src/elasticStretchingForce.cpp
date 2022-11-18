#include "elasticStretchingForce.h"
#include <iostream>

elasticStretchingForce::elasticStretchingForce(elasticRod &m_rod, timeStepper &m_stepper)
{
    rod = &m_rod;
    stepper = &m_stepper;

    f.setZero(3);
    Jss.setZero(7,7);
    Id3 << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    EA = rod->EA;
}

elasticStretchingForce::~elasticStretchingForce()
{
    ;
}

void elasticStretchingForce::computeFs()
{
    for (int i=0; i < rod->ne; i++)
    {
        epsX = rod->edgeLen(i) / rod->refLen(i) - 1.0;
        f = EA * (rod->tangent).row(i) * epsX;
        for (int k = 0; k < 3; k++)
        {
            ind = 4*i + k;
            stepper->addForce(ind, - f[k]); // subtracting elastic force

            ind = 4*(i + 1) + k;
            stepper->addForce(ind, f[k]); // adding elastic force
        }
    }
}

void elasticStretchingForce::computeJs()
{
    for (int i=0; i<rod->ne; i++)
    {
        len = rod->edgeLen[i];
        refLength = rod->refLen[i];

        dxx(0) = rod->x(4*i+4) - rod->x(4*i+0);
        dxx(1) = rod->x(4*i+5) - rod->x(4*i+1);
        dxx(2) = rod->x(4*i+6) - rod->x(4*i+2);

        u = dxx;
        v = u.transpose();
        M0= EA * ((1/refLength - 1/len) * Id3 + (1/len) * (u*v) / (u.norm() * u.norm()));

        Jss.block(0,0,3,3) =  - M0;
        Jss.block(4,4,3,3) =  - M0;
        Jss.block(4,0,3,3) =    M0;
        Jss.block(0,4,3,3) =    M0;

        for (int j = 0; j < 7; j++)
        {
            for (int k = 0; k < 7; k++)
            {
                ind1 = 4*i + j;
                ind2 = 4*i + k;
                stepper->addJacobian(ind1, ind2, - Jss(k,j));
            }
        }
    }
}
