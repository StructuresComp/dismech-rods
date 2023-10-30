#include "rodController.h"


rodController::rodController(const vector<shared_ptr<elasticRod>>& limbs) :
                             limbs(limbs), num_actuators(limbs.size()), current_time(0)
{
}


rodController::~rodController() = default;


// but we can also implement the timestepping here, for others to override if desired.
void rodController::updateTimestep(double dt)
{
    current_time += dt;
}