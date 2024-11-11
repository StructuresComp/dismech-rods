#ifndef ELASTIC_STRETCHING_FORCE_H
#define ELASTIC_STRETCHING_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class ElasticStretchingForce : public BaseForce
{
  public:
    explicit ElasticStretchingForce(const shared_ptr<SoftRobots>& m_soft_robots);
    ~ElasticStretchingForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

  private:
    double len, refLength;
    double epsX;
    Vector3d u;
    Vector3d dxx;
    Vector3d f;
    Matrix3d Id3;
    Matrix3d M0;
    Matrix<double, 1, 3> v;
    Matrix<double, 7, 7> Jss;

    double EA;
    int ind, ind1, ind2;
};

#endif  // ELASTIC_STRETCHING_FORCE_H
