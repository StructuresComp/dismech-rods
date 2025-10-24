#include "headless_sim_env.h"
#include "logging/base_logger.h"
#include "world.h"

HeadlessSimEnv::HeadlessSimEnv(const std::shared_ptr<World>& m_world,
                               const RenderParams& render_params,
                               const std::shared_ptr<BaseLogger>& logger)
    : BaseSimEnv(m_world, render_params, logger) {
}

HeadlessSimEnv::~HeadlessSimEnv() = default;

void HeadlessSimEnv::stepSimulation() {
    w_p->updateTimeStep();
    // try {
    //     w_p->updateTimeStep();
    // }
    // catch (std::runtime_error& excep) {
    //     std::cout << "Caught a std::runtime_error when trying to World->updateTimeStep: "
    //               << excep.what() << std::endl;
    //     std::cout << "Attempting clean shutdown..." << std::endl;
    //     cleanShutdown();
    //     return;
    // }

    if (is_logging) {
        logger_p->logWorldData();
    }

    cmdlineOutputHelper();
}

void HeadlessSimEnv::runSimulation() {
    while (w_p->simulationRunning()) {
        stepSimulation();
    }
}
