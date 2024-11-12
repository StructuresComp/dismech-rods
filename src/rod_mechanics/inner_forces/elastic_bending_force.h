#ifndef ELASTIC_BENDING_FORCE_H
#define ELASTIC_BENDING_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class ElasticBendingForce : public BaseForce
{
  public:
    explicit ElasticBendingForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~ElasticBendingForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    void jacobianComputation();

    int ci;
    double chi;
    double len;
    double kappa1, kappa2;
    double norm_e, norm_f;
    double norm2_e, norm2_f;
    double D2kappa1Dthetae2, D2kappa1Dthetaf2, D2kappa2Dthetae2, D2kappa2Dthetaf2;

    Vec3 t0, t1;
    Vec3 te, tf;
    Vec3 d1e, d1f, d2e, d2f;
    Vec3 tilde_t, tilde_d1, tilde_d2;
    Vec3 Dkappa1De, Dkappa1Df, Dkappa2De, Dkappa2Df;
    Vec3 kbLocal;
    Vec2 kappaL;
    Vec3 tmp;
    Vec3 D2kappa1DeDthetae, D2kappa1DeDthetaf, D2kappa1DfDthetae, D2kappa1DfDthetaf;
    Vec3 D2kappa2DeDthetae, D2kappa2DeDthetaf, D2kappa2DfDthetae, D2kappa2DfDthetaf;
    Vec2 temp;
    Vec3 m1e, m2e, m1f, m2f;
    VecX kappa11;
    VecX kappa22;
    VecX f;

    std::vector<std::shared_ptr<MatX>> gradKappa1s;
    std::vector<std::shared_ptr<MatX>> gradKappa2s;

    std::shared_ptr<MatX> gradKappa1;
    std::shared_ptr<MatX> gradKappa2;
    MatX relevantPart;
    MatX kappa;
    MatX DDkappa1, DDkappa2;
    MatX Jbb;

    std::vector<Mat2> EIMatrices;
    Mat3 Id3;
    Mat3 tt_o_tt;
    Mat3 tilde_d1_3d, tilde_d2_3d;
    Mat3 tf_c_d2t_o_tt, tt_o_tf_c_d2t, kb_o_d2e, d2e_o_kb;
    Mat3 D2kappa1De2;
    Mat3 te_c_d2t_o_tt, tt_o_te_c_d2t, kb_o_d2f, d2f_o_kb;
    Mat3 D2kappa1Df2;
    Mat3 D2kappa1DeDf, D2kappa1DfDe;
    Mat3 tf_c_d1t_o_tt, tt_o_tf_c_d1t, kb_o_d1e, d1e_o_kb;
    Mat3 D2kappa2De2;
    Mat3 te_c_d1t_o_tt, tt_o_te_c_d1t, kb_o_d1f, d1f_o_kb;
    Mat3 D2kappa2Df2;
    Mat3 D2kappa2DeDf;
    Mat3 D2kappa2DfDe;

    void crossMat(const Vec3& a, Mat3& b);
};

#endif  // ELASTIC_BENDING_FORCE_H
