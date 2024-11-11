#ifndef POSITION_LOGGER_H
#define POSITION_LOGGER_H

#include "base_logger.h"

class PositionLogger : public BaseLogger
{
  public:
    PositionLogger(string logfile_base, ofstream& df, int per);
    PositionLogger(string logfile_base, string logfile_suffix, ofstream& df, int per);
    ~PositionLogger();

  private:
    string getLogHeader() override;
    string getLogData() override;
};

#endif  // POSITION_LOGGER_H
