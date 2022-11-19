#ifndef ELASTICTWISTINGFORCE_H
#define ELASTICTWISTINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class elasticTwistingForce
{
public:
    elasticTwistingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper);
    ~elasticTwistingForce();
    void computeFt();
    void computeJt();

private:

    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;

    int ci, ind, ind1, ind2;
    double norm_e,norm_f;
    double norm2_e,norm2_f;
    double value,chi,milen;

    Vector3d t0,t1;
    Vector3d te,tf;
    Vector3d kbLocal;
    Vector3d tilde_t;
    VectorXd theta_f;
    VectorXd theta_e;
    VectorXd deltam;
    VectorXd gradTwistLocal;
    VectorXd getUndeformedTwist;
    VectorXd f;

    Matrix3d D2mDe2,D2mDf2,D2mDeDf,D2mDfDe;
    Matrix3d teMatrix;
    Matrix<double,11,11> J;
    Matrix<double,11,11> DDtwist;
    Matrix<double,11,11> Jtt;
    MatrixXd gradTwist;
    double GJ;
    
    void crossMat(const Vector3d &a,Matrix3d &b);
};

#endif
