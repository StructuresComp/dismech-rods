/**
 * BaseSimEnv.h
 *
 * Declarations for the abstract class BaseSimEnv.
 * A BaseSimEnv actually executes the DER simulation,
 * abstracting away the GUI if needed.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef BASE_SIM_ENV_H
#define BASE_SIM_ENV_H

// We'll need access to the World, and to Eigen classes.
#include "global_definitions.h"

class World;
class BaseLogger;

// Abstract class: stores the World and optionally the logger to use for that
// World.
class BaseSimEnv
{
  public:
    // TO-DO: add here...
    // max time until self-destruct
    // frequency of reporting to command line (per "VERBOSITY")

    BaseSimEnv(const std::shared_ptr<World>& m_world, const RenderParams& render_params,
               const std::shared_ptr<BaseLogger>& logger);
    virtual ~BaseSimEnv();

    /**
     * Setup function, called SEPARATELY, as needed according to specific
     * environment
     */
    // virtual void setup() = 0;

    /**
     * Start the simulation!
     */
    virtual void runSimulation() = 0;

    // Step the simulation one time step.
    virtual void stepSimulation() = 0;

  protected:
    // A helper, to make the command line output (via cmdline_per) general to
    // all simulation environments. Note, must be called manually in a child's
    // runSimulation! and annoyingly enough, to make compatible with GLUT's C
    // implementation, need to have a static method that takes in the World,
    // etc. to output.
    static void cmdlineOutputHelper(const std::shared_ptr<World>& s_world_p, int s_cmdline_per);
    // the non-static version will call the static one with our local variables.
    void cmdlineOutputHelper();

    // Helper to deal with clean shutdowns. Subclasses can override if desired,
    // but doing it in the superclass gives a basic example
    // similarly, because of openGL, we need a static version here that takes in
    // a pointer to the logger.
    static void cleanShutdown(std::shared_ptr<BaseLogger> s_logger_p, bool s_is_logging);
    void cleanShutdown();

    // Locals, see constructor for info
    std::shared_ptr<World> w_p = nullptr;
    int cmdline_per;

    // and a reference to the logger if specified.
    std::shared_ptr<BaseLogger> logger_p = nullptr;
    // if not specified, don't log
    bool is_logging;
};

#endif  // BASE_SIM_ENV
