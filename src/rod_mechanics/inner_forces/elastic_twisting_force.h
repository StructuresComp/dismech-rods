#ifndef ELASTIC_TWISTING_FORCE_H
#define ELASTIC_TWISTING_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class ElasticTwistingForce : public BaseForce
{
  public:
    explicit ElasticTwistingForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~ElasticTwistingForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    int ci, ind, ind1, ind2;
    double norm_e, norm_f;
    double norm2_e, norm2_f;
    double value, chi, milen;

    Vec3 t0, t1;
    Vec3 te, tf;
    Vec3 kbLocal;
    Vec3 tilde_t;
    std::shared_ptr<VecX> theta_f;
    std::shared_ptr<VecX> theta_e;
    std::shared_ptr<VecX> deltam;
    VecX gradTwistLocal;
    VecX getUndeformedTwist;
    VecX f;

    std::vector<std::shared_ptr<MatX>> gradTwists;
    std::vector<std::shared_ptr<VecX>> deltams;
    std::vector<std::shared_ptr<VecX>> theta_fs;
    std::vector<std::shared_ptr<VecX>> theta_es;

    Mat3 D2mDe2, D2mDf2, D2mDeDf, D2mDfDe;
    Mat3 teMatrix;
    SqMat<11> J;
    SqMat<11> DDtwist;
    SqMat<11> Jtt;
    std::shared_ptr<MatX> gradTwist;
    double GJ;

    void crossMat(const Vec3& a, Mat3& b);
};

#endif  // ELASTIC_TWISTING_FORCE_H
