#ifndef POSITION_LOGGER_H
#define POSITION_LOGGER_H

#include "base_logger.h"

class PositionLogger : public BaseLogger
{
  public:
    PositionLogger(std::string logfile_base, std::ofstream& df, int per);
    PositionLogger(std::string logfile_base, std::string logfile_suffix, std::ofstream& df,
                   int per);
    ~PositionLogger();

  private:
    std::string getLogHeader() override;
    std::string getLogData() override;
};

#endif  // POSITION_LOGGER_H
