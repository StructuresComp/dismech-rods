#include "headlessDERSimulationEnvironment.h"

headlessDERSimulationEnvironment::headlessDERSimulationEnvironment(
    const shared_ptr<world>& m_world, const renderParams& render_params,
    const shared_ptr<worldLogger>& logger)
    : derSimulationEnvironment(m_world, render_params, logger) {
}

headlessDERSimulationEnvironment::~headlessDERSimulationEnvironment() = default;

void headlessDERSimulationEnvironment::stepSimulation() {
    try {
        w_p->updateTimeStep();
    }
    catch (std::runtime_error& excep) {
        std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what()
                  << std::endl;
        std::cout << "Attempting clean shutdown..." << std::endl;
        cleanShutdown();
        return;
    }

    if (is_logging) {
        logger_p->logWorldData();
    }

    cmdlineOutputHelper();
}

void headlessDERSimulationEnvironment::runSimulation() {
    while (w_p->simulationRunning()) {
        stepSimulation();
    }
}
