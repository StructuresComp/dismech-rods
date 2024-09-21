#include "headlessDERSimulationEnvironment.h"


headlessDERSimulationEnvironment::headlessDERSimulationEnvironment(const shared_ptr<world>& m_world,
                                                                   const renderParams& render_params,
                                                                   const shared_ptr<worldLogger>& logger) :
	                                                               derSimulationEnvironment(m_world, render_params, logger)
{
}


headlessDERSimulationEnvironment::~headlessDERSimulationEnvironment() = default;


// Loop while world has yet to reach its final time.
void headlessDERSimulationEnvironment::runSimulation()
{
	// let our users know not to expect a GUI
    std::cout << "Using a headless simulation environment. Reporting back at an interval of " << cmdline_per << " msec." << std::endl;
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
            std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what() << std::endl;
            std::cout << "Attempting clean shutdown..." << std::endl;
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
