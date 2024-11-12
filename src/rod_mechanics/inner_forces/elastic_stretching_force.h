#ifndef ELASTIC_STRETCHING_FORCE_H
#define ELASTIC_STRETCHING_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class ElasticStretchingForce : public BaseForce
{
  public:
    explicit ElasticStretchingForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~ElasticStretchingForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    double len, refLength;
    double epsX;
    Vec3 u;
    Vec3 dxx;
    Vec3 f;
    Mat3 Id3;
    Mat3 M0;
    Mat<1, 3> v;
    SqMat<7> Jss;

    double EA;
    int ind, ind1, ind2;
};

#endif  // ELASTIC_STRETCHING_FORCE_H
