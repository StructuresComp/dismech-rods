#ifndef RODNODE_LOGGER_H
#define RODNODE_LOGGER_H

#include "worldLogger.h"


class rodNodeLogger : public worldLogger {
public:
    rodNodeLogger(string file_name_prefix, string logfile_base, ofstream& df, shared_ptr<world> w, int per);
    ~rodNodeLogger();
private:
    string getLogHeader() override;
    string getLogData() override;
};


#endif
