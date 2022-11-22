/**
 * headlessDERSimulationEnvironment.cpp
 *
 * Definitions for the concrete class headlessDERSimulationEnvironment.
 * Runs the DER simulation with no interface.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

// all includes should be in header
#include "headlessDERSimulationEnvironment.h"

// Constructors just call parents
headlessDERSimulationEnvironment::headlessDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per) :
									derSimulationEnvironment(m_world, m_cmdline_per)
{
}

headlessDERSimulationEnvironment::headlessDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, shared_ptr<worldLogger> m_logger) :
	derSimulationEnvironment(m_world, m_cmdline_per, m_logger)
{
}

headlessDERSimulationEnvironment::~headlessDERSimulationEnvironment()
{
}

// Loop while world has yet to reach its final time.
void headlessDERSimulationEnvironment::runSimulation()
{
	// let our users know not to expect a GUI
	if( verbosity >= 1){
		std::cout << "Using a headless simulation environment. Reporting back at an interval of " << cmdline_per << " msec." << std::endl;
	}
	// Before the simulation starts: log the x0 state.
	if(is_logging){
		logger_p->logWorldData();
	}
	// loop until done. The world knows its max time.
	while ( w_p->simulationRunning() > 0)
	{
		// catch convergence errors
		try {
			w_p->updateTimeStep(); // update time step
		}
		catch(std::runtime_error& excep){
			if(verbosity >= 1){
				std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what() << std::endl;
				std::cout << "Attempting clean shutdown..." << std::endl;
			}
			// superclass has the method
			cleanShutdown();
			// ugly to return here, but that's life
			return;
		}

		// log if specified.
		if(is_logging){
			logger_p->logWorldData();
		}
		// periodically report to the command line.
		cmdlineOutputHelper();
	}
}
