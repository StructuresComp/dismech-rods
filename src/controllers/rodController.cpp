/**
 * rodController.cpp
 * 
 * Some minimal definitions for functions in the abstract 
 * class rodController. TO-DO: what to include here??
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodController.h"

// Constructor just stores the number of actuators.
rodController::rodController(int numAct) : numActuators(numAct), current_time(0.0)
{
}

rodController::~rodController()
{
}

// but we can also implement the timestepping here, for others to override if desired.
void rodController::updateTimestep(double dt)
{
    current_time += dt;
}