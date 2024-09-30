/**
 * derSimulationEnvironment.h
 *
 * Declarations for the abstract class derSimulationEnvironment.
 * A derSimulationEnvironment actually executes the DER simulation,
 * abstracting away the GUI if needed.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef DER_SIMULATION_ENVIRONMENT_H
#define DER_SIMULATION_ENVIRONMENT_H

// We'll need access to the world, and to Eigen classes.
#include "globalDefinitions.h"
#include "world.h"
// The logging framework is separate from world.
#include "logging/worldLogger.h"

// Abstract class: stores the world and optionally the logger to use for that world.
class derSimulationEnvironment
{
    public:

    // TO-DO: add here...
    // max time until self-destruct
    // frequency of reporting to command line (per "VERBOSITY")

    derSimulationEnvironment(const shared_ptr<world>& m_world, const renderParams& render_params,
                             const shared_ptr<worldLogger>& logger);
    virtual ~derSimulationEnvironment();

    /**
     * Setup function, called SEPARATELY, as needed according to specific environment
     */
    // virtual void setup() = 0;

    /**
     * Start the simulation!
     */
    virtual void runSimulation() = 0;

    // Step the simulation one time step.
    virtual void stepSimulation() = 0;

    protected:

    // A helper, to make the command line output (via cmdline_per) general to all simulation environments.
    // Note, must be called manually in a child's runSimulation!
    // and annoyingly enough, to make compatible with GLUT's C implementation, need to have a
    // static method that takes in the world, etc. to output.
    static void cmdlineOutputHelper(const shared_ptr<world>& s_world_p, int s_cmdline_per);
    // the non-static version will call the static one with our local variables.
    void cmdlineOutputHelper();

    // Helper to deal with clean shutdowns. Subclasses can override if desired,
    // but doing it in the superclass gives a basic example
    // similarly, because of openGL, we need a static version here that takes in a pointer to the logger.
    static void cleanShutdown(shared_ptr<worldLogger> s_logger_p, bool s_is_logging);
    void cleanShutdown();


    // Locals, see constructor for info
    shared_ptr<world> w_p = nullptr;
    int cmdline_per;

    // and a reference to the logger if specified.
    shared_ptr<worldLogger> logger_p = nullptr;
    // if not specified, don't log
    bool is_logging;
};

#endif
