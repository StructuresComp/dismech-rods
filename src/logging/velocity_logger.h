#ifndef VELOCITY_LOGGER_H
#define VELOCITY_LOGGER_H

#include "base_logger.h"

class VelocityLogger : public BaseLogger
{
  public:
    VelocityLogger(string logfile_base, ofstream& df, int per);
    VelocityLogger(string logfile_base, string logfile_suffix, ofstream& df, int per);
    ~VelocityLogger();

  private:
    string getLogHeader() override;
    string getLogData() override;
};

#endif  // VELOCITY_LOGGER_H
