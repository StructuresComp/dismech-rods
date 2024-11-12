#ifndef FORCE_CONTAINER_H
#define FORCE_CONTAINER_H

#include "global_definitions.h"

class BaseForce;
class BaseTimeStepper;
class ContactForce;
class FloorContactForce;

class ForceContainer
{
  public:
    ForceContainer();
    explicit ForceContainer(const std::vector<std::shared_ptr<BaseForce>>& m_forces);
    ~ForceContainer();

    void computeForces(double dt);
    void computeForcesAndJacobian(double dt);

    void setupForceStepperAccess(const std::shared_ptr<BaseTimeStepper>& stepper);

    void addForce(const std::shared_ptr<BaseForce>& force);

    std::shared_ptr<ContactForce> cf;
    std::shared_ptr<FloorContactForce> ff;

  private:
    std::vector<std::shared_ptr<BaseForce>> forces;
};

#endif  // FORCE_CONTAINER_H
