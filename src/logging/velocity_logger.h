#ifndef VELOCITY_LOGGER_H
#define VELOCITY_LOGGER_H

#include "base_logger.h"

class VelocityLogger : public BaseLogger
{
  public:
    VelocityLogger(std::string logfile_base, std::ofstream& df, int per);
    VelocityLogger(std::string logfile_base, std::string logfile_suffix, std::ofstream& df,
                   int per);
    ~VelocityLogger();

  private:
    std::string getLogHeader() override;
    std::string getLogData() override;
};

#endif  // VELOCITY_LOGGER_H
