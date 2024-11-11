#ifndef FORCE_CONTAINER_H
#define FORCE_CONTAINER_H

#include "base_force.h"
#include "external_forces/contact_force.h"
#include "external_forces/floor_contact_force.h"

class ForceContainer
{
  public:
    ForceContainer();
    explicit ForceContainer(const vector<shared_ptr<BaseForce>>& m_forces);
    ~ForceContainer();

    void computeForces(double dt);
    void computeForcesAndJacobian(double dt);

    void setupForceStepperAccess(const shared_ptr<BaseTimeStepper>& stepper);

    void addForce(const shared_ptr<BaseForce>& force);

    shared_ptr<ContactForce> cf;
    shared_ptr<FloorContactForce> ff;

  private:
    vector<shared_ptr<BaseForce>> forces;
};

#endif  // FORCE_CONTAINER_H
