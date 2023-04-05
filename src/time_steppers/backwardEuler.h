#ifndef BACKWARDEULER_H
#define BACKWARDEULER_H

#include "implicitTimeStepper.h"

class backwardEuler : public implicitTimeStepper
{
public:
    backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs);
    ~backwardEuler();

    void integrator() override;

};


#endif
