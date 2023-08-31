#include "rodNodeLogger.h"
#include "world.h"


rodNodeLogger::rodNodeLogger(std::string logfile_base, std::ofstream &df, int per) :
                             worldLogger("node", logfile_base, df, per)
{
}


rodNodeLogger::~rodNodeLogger() = default;

string rodNodeLogger::getLogHeader() {
    return "";
}

string rodNodeLogger::getLogData() {
    ostringstream logdata;
    logdata << world_ptr->getCurrentTime();
    for (auto& limb : world_ptr->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vector3d v = limb->getVertex(i);
            logdata << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return logdata.str();
}