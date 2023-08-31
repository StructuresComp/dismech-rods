#ifndef ELASTICBENDINGFORCE_H
#define ELASTICBENDINGFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class elasticBendingForce : public baseForce
{
public:
    elasticBendingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                        const vector<shared_ptr<elasticJoint>>& m_joints);
    ~elasticBendingForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    void JacobianComputation();

    int ci;
    double chi;
    double len;
    double kappa1, kappa2;
    double norm_e, norm_f;
    double norm2_e, norm2_f;
    double D2kappa1Dthetae2, D2kappa1Dthetaf2, D2kappa2Dthetae2, D2kappa2Dthetaf2;
    
    Vector3d t0, t1;
    Vector3d te, tf;
    Vector3d d1e, d1f, d2e, d2f;
    Vector3d tilde_t, tilde_d1, tilde_d2;
    Vector3d Dkappa1De, Dkappa1Df, Dkappa2De, Dkappa2Df;
    Vector3d kbLocal;
    Vector2d kappaL;
    Vector3d tmp;
    Vector3d D2kappa1DeDthetae, D2kappa1DeDthetaf, D2kappa1DfDthetae, D2kappa1DfDthetaf;
    Vector3d D2kappa2DeDthetae, D2kappa2DeDthetaf, D2kappa2DfDthetae, D2kappa2DfDthetaf;
    Vector2d temp;
    Vector3d m1e, m2e, m1f, m2f;
    VectorXd kappa11;
    VectorXd kappa22;
    VectorXd f;

    vector<shared_ptr<MatrixXd>> gradKappa1s;
    vector<shared_ptr<MatrixXd>> gradKappa2s;

    shared_ptr<MatrixXd> gradKappa1;
    shared_ptr<MatrixXd> gradKappa2;
    MatrixXd relevantPart;
    MatrixXd kappa;
    MatrixXd DDkappa1, DDkappa2;
    MatrixXd Jbb;

    vector<Matrix2d> EIMatrices;
    Matrix3d Id3;
    Matrix3d tt_o_tt;
    Matrix3d tilde_d1_3d, tilde_d2_3d;
    Matrix3d tf_c_d2t_o_tt, tt_o_tf_c_d2t, kb_o_d2e, d2e_o_kb;
    Matrix3d D2kappa1De2;
    Matrix3d te_c_d2t_o_tt, tt_o_te_c_d2t, kb_o_d2f, d2f_o_kb;
    Matrix3d D2kappa1Df2;
    Matrix3d D2kappa1DeDf, D2kappa1DfDe;
    Matrix3d tf_c_d1t_o_tt, tt_o_tf_c_d1t, kb_o_d1e, d1e_o_kb;
    Matrix3d D2kappa2De2;
    Matrix3d te_c_d1t_o_tt, tt_o_te_c_d1t, kb_o_d1f, d1f_o_kb;
    Matrix3d D2kappa2Df2;
    Matrix3d D2kappa2DeDf;
    Matrix3d D2kappa2DfDe;
        
    void crossMat(const Vector3d &a,Matrix3d &b);
};

#endif
