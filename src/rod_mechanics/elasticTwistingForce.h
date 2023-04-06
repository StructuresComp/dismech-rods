#ifndef ELASTICTWISTINGFORCE_H
#define ELASTICTWISTINGFORCE_H

#include "baseForce.h"

class baseTimeStepper;

class elasticTwistingForce : public baseForce
{
public:
    elasticTwistingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                         const vector<shared_ptr<elasticJoint>>& m_joints);
    ~elasticTwistingForce() override;
    void computeFt();
    void computeJt();

private:
    int ci, ind, ind1, ind2;
    double norm_e,norm_f;
    double norm2_e,norm2_f;
    double value,chi,milen;

    Vector3d t0,t1;
    Vector3d te,tf;
    Vector3d kbLocal;
    Vector3d tilde_t;
    shared_ptr<VectorXd> theta_f;
    shared_ptr<VectorXd> theta_e;
    shared_ptr<VectorXd> deltam;
    VectorXd gradTwistLocal;
    VectorXd getUndeformedTwist;
    VectorXd f;

    vector<shared_ptr<MatrixXd>> gradTwists;
    vector<shared_ptr<VectorXd>> deltams;
    vector<shared_ptr<VectorXd>> theta_fs;
    vector<shared_ptr<VectorXd>> theta_es;

    Matrix3d D2mDe2,D2mDf2,D2mDeDf,D2mDfDe;
    Matrix3d teMatrix;
    Matrix<double,11,11> J;
    Matrix<double,11,11> DDtwist;
    Matrix<double,11,11> Jtt;
    shared_ptr<MatrixXd> gradTwist;
    double GJ;
    
    void crossMat(const Vector3d &a,Matrix3d &b);
};

#endif
