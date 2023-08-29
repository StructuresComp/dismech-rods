#ifndef ELASTICSTRETCHINGFORCE_H
#define ELASTICSTRETCHINGFORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class elasticStretchingForce : public baseForce
{
public:
    elasticStretchingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                           const vector<shared_ptr<elasticJoint>>& m_joints);
    ~elasticStretchingForce() override;
    void computeFs();
    void computeJs();

private:
    double len, refLength;
    double epsX;
    Vector3d u;
    Vector3d dxx;
    Vector3d f;
    Matrix3d Id3;
    Matrix3d M0;
    Matrix<double,1,3> v;
    Matrix<double,7,7> Jss;
    
    double EA;
    int ind, ind1, ind2;	
};

#endif
