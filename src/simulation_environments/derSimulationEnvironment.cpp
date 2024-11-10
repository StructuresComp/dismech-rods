#include "derSimulationEnvironment.h"

derSimulationEnvironment::derSimulationEnvironment(const shared_ptr<world>& m_world,
                                                   const renderParams& render_params,
                                                   const shared_ptr<worldLogger>& logger)
    : w_p(m_world), cmdline_per(render_params.cmd_line_per), logger_p(logger),
      is_logging(logger != nullptr) {
    if (is_logging) {
        logger_p->world_ptr = w_p;
        logger_p->setup();
    }
}

derSimulationEnvironment::~derSimulationEnvironment() = default;

void derSimulationEnvironment::cmdlineOutputHelper(const shared_ptr<world>& s_world_p,
                                                   int s_cmdline_per) {
    if (s_cmdline_per == 0)
        return;
    if (s_world_p->getTimeStep() % s_cmdline_per == 0) {
        s_world_p->printSimData();
    }
}

void derSimulationEnvironment::cmdlineOutputHelper() {
    // call the static method with our variables
    cmdlineOutputHelper(w_p, cmdline_per);
}

void derSimulationEnvironment::cleanShutdown(shared_ptr<worldLogger> s_logger_p,
                                             bool s_is_logging) {
    // Deletes will be taken care of by smart pointers.
    // what's more important is pruning the log files if needed,
    // we don't want empty ones.
    // partially filled ones are OK though
    if (s_is_logging) {
        s_logger_p->pruneEmptyLog();
    }
}

void derSimulationEnvironment::cleanShutdown() {
    cleanShutdown(logger_p, is_logging);
}
