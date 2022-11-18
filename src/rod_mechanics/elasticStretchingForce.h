#ifndef ELASTICSTRETCHINGFORCE_H
#define ELASTICSTRETCHINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class elasticStretchingForce
{
public:
    elasticStretchingForce(elasticRod &m_rod, timeStepper &m_stepper);
    ~elasticStretchingForce();
    void computeFs();
    void computeJs();

private:
    elasticRod *rod;
    timeStepper *stepper;

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
