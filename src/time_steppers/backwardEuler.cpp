#include "backwardEuler.h"

backwardEuler::backwardEuler(const vector<shared_ptr<elasticRod>>& m_limbs) : implicitTimeStepper(m_limbs) {

}

backwardEuler::~backwardEuler() {

}


void backwardEuler::integrator()
{
    pardisoSolver();
    // TODO: move the newton's method stuff here
}
