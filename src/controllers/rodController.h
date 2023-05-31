/**
 * rodController.h
 * 
 * Declarations for the abstract class rodController.
 * A rodController calculates some inputs (as-yet-unspecified) to a DER, 
 * using state feedback (pass in a pointer to the whole rod and decide what to use later).
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_CONTROLLER_H
#define ROD_CONTROLLER_H

// Some constants used throughout this program
#include "../global_const.h"

// Since the world will be owning this, don't include world,
// but instead the rod alone.
#include "../rod_mechanics/elasticRod.h"
// may want to return Eigen objects too??

class rodController
{

    public:

    /**
     * Constructor for a rodController.
     * Store the number of actuators as a local variable
     * TO-DO: THIS IS ALSO READ AS A TEXT FILE VARIABLE IN WORLD!!!! SHOULDN'T STORE TWICE!
     */
    rodController(int numAct);
    ~rodController();

    /**
     * Calculate the desired control input, u(x).
     * We do need to specify a return type. For now, doing PWM, that's "on/off".
     * Subclass must implement.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    virtual std::vector<int> getU(shared_ptr<elasticRod> rod_p) = 0;

    // Controllers can be timestepped so they have access to the world's time.
    // derived classes can override if needed.
    virtual void updateTimestep(double dt);

    protected:

    // Store the number of actuators in the robot.
    // Makes it easier to initialize u(x) and also to run verification checks (TO-DO: THIS)
    int numActuators;

    // and the overall simulation time
    double current_time;
};

#endif //ROD_CONTROLLER_H