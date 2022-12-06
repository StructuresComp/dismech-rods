#ifndef ELASTICSTRETCHINGFORCE_H
#define ELASTICSTRETCHINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "Joint.h"
#include "timeStepper.h"

class elasticStretchingForce
{
public:
    elasticStretchingForce(vector<shared_ptr<elasticRod>> m_limbs, vector<shared_ptr<Joint>> m_joints,
                           shared_ptr<timeStepper> m_stepper);
    ~elasticStretchingForce();
    void computeFs();
    void computeJs();

private:
    vector<shared_ptr<elasticRod>> limbs;
    vector<shared_ptr<Joint>> joints;
    shared_ptr<elasticRod> rod;
    shared_ptr<timeStepper> stepper;

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
