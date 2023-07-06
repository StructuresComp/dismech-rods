/**
 * derSimulationEnvironment.cpp
 *
 * Definitions for the abstract class derSimulationEnvironment.
 * A derSimulationEnvironment actually executes the DER simulation,
 * abstracting away the GUI if needed.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

// all includes in this' header
#include "derSimulationEnvironment.h"

// Constructors: store locals and set the boolean for logging or not

derSimulationEnvironment::derSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per) :
                                w_p(m_world), cmdline_per(m_cmdline_per), logger_p(NULL), is_logging(false)
{
}

derSimulationEnvironment::derSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, shared_ptr<worldLogger> m_logger) :
                                w_p(m_world), cmdline_per(m_cmdline_per), logger_p(m_logger), is_logging(true)
{
}

derSimulationEnvironment::~derSimulationEnvironment() = default;

// the static one does the heavy lifting
void derSimulationEnvironment::cmdlineOutputHelper(shared_ptr<world> s_world_p, int s_cmdline_per)
{
    if (s_world_p->getTimeStep() % s_cmdline_per == 0) {
        s_world_p->printSimData();
    }

    // TODO: comment this out for nwo until we know what we're actually logging

//    // If the current simulation time for the world is an interval of our period,
//    // output a little check-in message.
//    // verbosity should overrride here.
//    if(verbosity >= 1){
//        // note that cmdline_per is implicity in msec here.
//        if(s_world_p->getTimeStep() % s_cmdline_per == 0) {
//            // Like in the logger_p, output the current simulation time, plus the states.
//            std::cout << "Simulation time: " << s_world_p->getCurrentTime() << ", State: [";
//            // X, Y
//            Vector2d com = s_world_p->getRodP()->getCOM();
//            std::cout << com[0] << ", " << com[1] << ", ";
//            // Theta, see elasticRod for more info
//            std::cout << s_world_p->getRodP()->getRBRotation() << ", ";
//            // dot X, dot Y (velocities)
//            Vector2d com_vel = s_world_p->getRodP()->getVelocityCOM();
//            std::cout << com_vel[0] << ", " << com_vel[1] << ", ";
//            // and angular velocity too.
//            std::cout << s_world_p->getRodP()->getVelocityAngular();
//            std::cout << "]" << std::endl;
//        }
//    }
}

void derSimulationEnvironment::cmdlineOutputHelper()
{
    // call the static method with our variables
    cmdlineOutputHelper(w_p, cmdline_per);
}

void derSimulationEnvironment::cleanShutdown(shared_ptr<worldLogger> s_logger_p, bool s_is_logging)
{
    // Deletes will be taken care of by smart pointers.
    // what's more important is pruning the log files if needed,
    // we don't want empty ones.
    // partially filled ones are OK though
    if( s_is_logging ){
        s_logger_p->pruneEmptyLog();
    }
}

void derSimulationEnvironment::cleanShutdown()
{
    cleanShutdown(logger_p, is_logging);
}


// Childen must define runSimulation.