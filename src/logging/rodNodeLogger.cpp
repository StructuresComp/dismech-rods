#include "rodNodeLogger.h"


rodNodeLogger::rodNodeLogger(std::string file_name_prefix, std::string logfile_base, std::ofstream &df,
                             shared_ptr<world> w, int per) :
                             worldLogger(file_name_prefix, logfile_base, df, w, per)
{
}


rodNodeLogger::~rodNodeLogger() = default;

string rodNodeLogger::getLogHeader() {
    return "";
}

string rodNodeLogger::getLogData() {
    ostringstream logdata;
    logdata << m_world_p->getCurrentTime();
    for (auto& limb : m_world_p->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            Vector3d v = limb->getVertex(i);
            logdata << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return logdata.str();
}