/**
 * rodEmptyController.h
 * 
 * Definition(s) for the concrete class rodEmptyController.
 * Actuation always off.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodEmptyController.h"

// Constructor and destructor just call parents
rodEmptyController::rodEmptyController(int numAct) : rodController(numAct)
{
}

rodEmptyController::~rodEmptyController()
{
}

// Implementation of the controller.
std::vector<int> rodEmptyController::getU(shared_ptr<elasticRod> rod_p)
{
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    return u;
}