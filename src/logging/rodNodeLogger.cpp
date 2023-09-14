#include "rodNodeLogger.h"

#include <utility>
#include "world.h"


rodNodeLogger::rodNodeLogger(std::string logfile_base, std::ofstream &df, int per) :
                             worldLogger("node", std::move(logfile_base), df, per)
{
}


rodNodeLogger::rodNodeLogger(std::string logfile_base, std::string logfile_suffix, std::ofstream &df, int per) :
        worldLogger("node", std::move(logfile_suffix), std::move(logfile_base), df, per)
{
}


rodNodeLogger::~rodNodeLogger() = default;

string rodNodeLogger::getLogHeader() {
    return "";
}

string rodNodeLogger::getLogData() {
    ostringstream log_data;
    log_data << world_ptr->getCurrentTime();
    for (const auto& limb : world_ptr->soft_robots->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vector3d v = limb->getVertex(i);
            log_data << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return log_data.str();
}