#include "elasticTwistingForce.h"

elasticTwistingForce::elasticTwistingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper)
{
    rod = m_rod;
    stepper = m_stepper;

    gradTwist = MatrixXd::Zero(rod->nv,11);
    deltam = VectorXd::Zero(rod->ne);
    theta_f = VectorXd::Zero(rod->ne);
    theta_e = VectorXd::Zero(rod->ne);

    DDtwist.setZero(11,11);
    Jtt.setZero(11,11);
    gradTwistLocal.setZero(11);
    f.setZero(11);

    GJ = rod->GJ;    
}

elasticTwistingForce::~elasticTwistingForce()
{
    ;
}

void elasticTwistingForce::computeFt()
{
    for (int i = 0; i < rod->ne; i++)
    {
        theta_f(i) = rod->x(4*i+3);
    }

    for (int i = 0; i<rod->ne; i++)
    {
        if (i==0)
            theta_e(i)=0;
        else
            theta_e(i)=theta_f(i-1);
    }

    deltam = theta_f-theta_e;

    for(int i = 1; i < rod->ne; i++)
    {
        norm_e = rod->edgeLen(i-1);
        norm_f = rod->edgeLen(i);
        gradTwist.row(i).segment(0,3) = -0.5 / norm_e * (rod->kb).row(i);
        gradTwist.row(i).segment(8,3) = 0.5 / norm_f * (rod->kb).row(i);
        gradTwist.row(i).segment(4,3) = -(gradTwist.row(i).segment(0,3)+gradTwist.row(i).segment(8,3));
        gradTwist(i, 3) = -1;
        gradTwist(i, 7) =  1;
    }

    for(int i = 1; i < rod->ne; i++)
    {
        value = GJ / rod->voronoiLen(i) * (deltam(i) + rod->refTwist (i) - rod->undeformedTwist(i));
        ci = 4*i-4;
        f = -value * gradTwist.row(i);
        for (int k = 0; k < 11; k++)
        {
            ind = ci + k;
            stepper->addForce(ind, -f[k]); // subtracting elastic force
        }
    }
}

void elasticTwistingForce::computeJt()
{
    for (int i = 1; i < rod->ne; i++)
    {
        norm_e = rod->edgeLen(i-1);
        norm_f = rod->edgeLen(i);
        te = rod->tangent.row(i-1);
        tf = rod->tangent.row(i);

        norm2_e=norm_e*norm_e;
        norm2_f=norm_f*norm_f;

        kbLocal= (rod->kb).row(i);

        chi=1.0+te.dot(tf);
        tilde_t=(te+tf)/chi;

        crossMat(te,teMatrix);

        D2mDe2 = -0.25 / norm2_e * (kbLocal * (te+tilde_t).transpose()
            + (te+tilde_t) * kbLocal.transpose());
        D2mDf2 = -0.25 / norm2_f * (kbLocal * (tf+tilde_t).transpose()
            + (tf+tilde_t) * kbLocal.transpose());
        D2mDeDf = 0.5  / (norm_e*norm_f) * (2.0 / chi * teMatrix
            - kbLocal*tilde_t.transpose());
        D2mDfDe = D2mDeDf.transpose();

        DDtwist.block(0,0,3,3) = D2mDe2;
        DDtwist.block(0,4,3,3) =-D2mDe2 + D2mDeDf;
        DDtwist.block(4,0,3,3) =-D2mDe2 + D2mDfDe;
        DDtwist.block(4,4,3,3) = D2mDe2 - ( D2mDeDf + D2mDfDe ) + D2mDf2;
        DDtwist.block(0,8,3,3) =-D2mDeDf;
        DDtwist.block(8,0,3,3) =-D2mDfDe;
        DDtwist.block(8,4,3,3) = D2mDfDe - D2mDf2;
        DDtwist.block(4,8,3,3) = D2mDeDf - D2mDf2;
        DDtwist.block(8,8,3,3) = D2mDf2;

        gradTwistLocal = gradTwist.row(i);
        
        milen = -1/rod->voronoiLen(i);

        Jtt = GJ * milen * ((deltam(i)+rod->refTwist(i) - rod->undeformedTwist(i)) 
            * DDtwist + gradTwistLocal * gradTwistLocal.transpose());


        for (int j = 0; j < 11; j++)
        {
            for (int k = 0; k < 11; k++)
            {
                ind1 = 4*i - 4 + j;
                ind2 = 4*i - 4 + k;
                stepper->addJacobian(ind1, ind2, - Jtt(k,j));

            }
        }
        
    }
}

// Utility
void elasticTwistingForce::crossMat(const Vector3d &a,Matrix3d &b)
{
    b << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
}
