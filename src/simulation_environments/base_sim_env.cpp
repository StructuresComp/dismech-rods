#include "base_sim_env.h"

BaseSimEnv::BaseSimEnv(const std::shared_ptr<World>& m_world, const RenderParams& render_params,
                       const std::shared_ptr<BaseLogger>& logger)
    : w_p(m_world), cmdline_per(render_params.cmd_line_per), logger_p(logger),
      is_logging(logger != nullptr) {
    if (is_logging) {
        logger_p->world_ptr = w_p;
        logger_p->setup();
    }
}

BaseSimEnv::~BaseSimEnv() = default;

void BaseSimEnv::cmdlineOutputHelper(const std::shared_ptr<World>& s_world_p, int s_cmdline_per) {
    if (s_cmdline_per == 0)
        return;
    if (s_world_p->getTimeStep() % s_cmdline_per == 0) {
        s_world_p->printSimData();
    }
}

void BaseSimEnv::cmdlineOutputHelper() {
    // call the static method with our variables
    cmdlineOutputHelper(w_p, cmdline_per);
}

void BaseSimEnv::cleanShutdown(std::shared_ptr<BaseLogger> s_logger_p, bool s_is_logging) {
    // Deletes will be taken care of by smart pointers.
    // what's more important is pruning the log files if needed,
    // we don't want empty ones.
    // partially filled ones are OK though
    if (s_is_logging) {
        s_logger_p->pruneEmptyLog();
    }
}

void BaseSimEnv::cleanShutdown() {
    cleanShutdown(logger_p, is_logging);
}
