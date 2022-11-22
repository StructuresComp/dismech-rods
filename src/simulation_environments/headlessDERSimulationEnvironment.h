/**
 * headlessDERSimulationEnvironment.h
 *
 * Declarations for the concrete class headlessDERSimulationEnvironment.
 * Runs the DER simulation with no interface.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef HEADLESS_DER_SIMULATION_ENVIRONMENT_H
#define HEADLESS_DER_SIMULATION_ENVIRONMENT_H

// Subclass of simulation environment
#include "derSimulationEnvironment.h"

class headlessDERSimulationEnvironment : public derSimulationEnvironment
{
    public:

    /**
     * Constructors do the following:
     * (1) call the parents
     */
    headlessDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per);
    headlessDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, shared_ptr<worldLogger> m_logger);
    ~headlessDERSimulationEnvironment();

    /**
     * Setup function, called SEPARATELY.
     */
    // void setup();

    /**
     * Start the simulation!
     */
    void runSimulation();

    protected:

    // nothing here, simple class doesn't need helpers.

};

#endif // HEADLESS_DER_SIMULATION_ENVIRONMENT_H