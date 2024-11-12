#include "position_logger.h"

#include "world.h"
#include <utility>

PositionLogger::PositionLogger(std::string logfile_base, std::ofstream& df, int per)
    : BaseLogger("node", std::move(logfile_base), df, per) {
}

PositionLogger::PositionLogger(std::string logfile_base, std::string logfile_suffix,
                               std::ofstream& df, int per)
    : BaseLogger("node", std::move(logfile_suffix), std::move(logfile_base), df, per) {
}

PositionLogger::~PositionLogger() = default;

std::string PositionLogger::getLogHeader() {
    return "";
}

std::string PositionLogger::getLogData() {
    std::ostringstream log_data;
    log_data << world_ptr->getCurrentTime();
    for (const auto& limb : world_ptr->soft_robots->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vec3 v = limb->getVertex(i);
            log_data << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return log_data.str();
}
