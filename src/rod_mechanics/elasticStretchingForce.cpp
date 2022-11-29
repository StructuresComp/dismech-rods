#include "elasticStretchingForce.h"
#include <iostream>

elasticStretchingForce::elasticStretchingForce(vector<shared_ptr<elasticRod>> m_limbs, shared_ptr<timeStepper> m_stepper)
{
    limbs = m_limbs;
    stepper = m_stepper;

    f.setZero(3);
    Jss.setZero(7,7);
    Id3 << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
//    EA = rod->EA;
}

elasticStretchingForce::~elasticStretchingForce()
{
    ;
}

void elasticStretchingForce::computeFs()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {

        for (int i = 0; i < limb->ne; i++)
        {
            epsX = limb->edgeLen(i) / limb->refLen(i) - 1.0;
            f = limb->EA * (limb->tangent).row(i) * epsX;
            for (int k = 0; k < 3; k++)
            {
                ind = 4*i + k;
                stepper->addForce(ind, - f[k], limb_idx); // subtracting elastic force

                ind = 4*(i + 1) + k;
                stepper->addForce(ind, f[k], limb_idx); // adding elastic force
            }
        }
        limb_idx++;
    }
}

void elasticStretchingForce::computeJs()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        for (int i = 0; i < limb->ne; i++)
        {
            len = limb->edgeLen[i];
            refLength = limb->refLen[i];

            dxx(0) = limb->x(4*i+4) - limb->x(4*i+0);
            dxx(1) = limb->x(4*i+5) - limb->x(4*i+1);
            dxx(2) = limb->x(4*i+6) - limb->x(4*i+2);

            u = dxx;
            v = u.transpose();
            M0= limb->EA * ((1/refLength - 1/len) * Id3 + (1/len) * (u*v) / (u.norm() * u.norm()));

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
                    stepper->addJacobian(ind1, ind2, - Jss(k,j), limb_idx);
                }
            }
        }
        limb_idx++;
    }
}
