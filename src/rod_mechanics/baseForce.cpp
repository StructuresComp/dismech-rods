#include "baseForce.h"

baseForce::baseForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                     const vector<shared_ptr<elasticJoint>>& m_joints) :
                     limbs(m_limbs), joints(m_joints) {

}

baseForce::~baseForce() = default;

void baseForce::setTimeStepper(shared_ptr<baseTimeStepper> m_stepper) {
    stepper = m_stepper;
}
