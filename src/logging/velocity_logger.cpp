#include "velocity_logger.h"

#include "world.h"
#include <utility>

VelocityLogger::VelocityLogger(std::string logfile_base, std::ofstream& df, int per)
    : BaseLogger("velocities", std::move(logfile_base), df, per) {
}

VelocityLogger::VelocityLogger(std::string logfile_base, std::string logfile_suffix,
                               std::ofstream& df, int per)
    : BaseLogger("velocities", std::move(logfile_suffix), std::move(logfile_base), df, per) {
}

VelocityLogger::~VelocityLogger() = default;

std::string VelocityLogger::getLogHeader() {
    return "";
}

std::string VelocityLogger::getLogData() {
    std::ostringstream log_data;
    log_data << world_ptr->getCurrentTime();
    for (const auto& limb : world_ptr->soft_robots->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vec3 v = limb->getVelocity(i);
            log_data << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return log_data.str();
}
