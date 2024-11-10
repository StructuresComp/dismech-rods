#ifndef VELOCITY_LOGGER_H
#define VELOCITY_LOGGER_H

#include "worldLogger.h"

class velocityLogger : public worldLogger
{
  public:
    velocityLogger(string logfile_base, ofstream& df, int per);
    velocityLogger(string logfile_base, string logfile_suffix, ofstream& df, int per);
    ~velocityLogger();

  private:
    string getLogHeader() override;
    string getLogData() override;
};

#endif
