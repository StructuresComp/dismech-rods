#include "velocity_logger.h"

#include "world.h"
#include <utility>

VelocityLogger::VelocityLogger(string logfile_base, ofstream& df, int per)
    : BaseLogger("velocities", std::move(logfile_base), df, per) {
}

VelocityLogger::VelocityLogger(string logfile_base, string logfile_suffix, ofstream& df, int per)
    : BaseLogger("velocities", std::move(logfile_suffix), std::move(logfile_base), df, per) {
}

VelocityLogger::~VelocityLogger() = default;

string VelocityLogger::getLogHeader() {
    return "";
}

string VelocityLogger::getLogData() {
    ostringstream log_data;
    log_data << world_ptr->getCurrentTime();
    for (const auto& limb : world_ptr->soft_robots->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vector3d v = limb->getVelocity(i);
            log_data << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return log_data.str();
}
