/**
 * BaseLogger.h
 *
 * Declarations for the abstract class BaseLogger.
 * A BaseLogger publishes (writes to a file) certain data
 * from a World (DER simulation.)
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef BASE_LOGGER_H
#define BASE_LOGGER_H

// Only operating on a DER World.
class World;
// Some constants used throughout this program
#include "filesystem_finder.hpp"
#include "global_definitions.h"

// the C++ standard library
#include <fstream>  // for writing to a file

// Abstract class: although we'll do a constructor to set the
// path prefix variable, children have to define what to log and how.
class BaseLogger
{
  public:
    /**
     * Constructor for a BaseLogger.
     * @param fileNamePrefix a string to prepend to the file name
     * @param logfile_base the base folder for the logs (actual folder will be
     * organized by year, month, day)
     * @param df ref to an ofstream to use to write to the data file
     * @param per integer, period to write to log file (only once per this many
     * samples)
     */
    BaseLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, int per);
    BaseLogger(std::string fileNamePrefix, std::string file_name_suffix, std::string logfile_base,
               std::ofstream& df, int per);
    ~BaseLogger();

    /**
     * Setup function, separate from constructor, for various tasks
     * that are prevented from taking place before construction, e.g. init-ing
     * the log file. Note, calls a secondary (abstract) function for the
     * children to do whatever else they'd need
     */
    void setup();

    shared_ptr<World> world_ptr;

    /**
     * Log data, used by the caller.
     * Writes to file according to getLogData at period.
     */
    void logWorldData();

    /**
     * For a clean shutdown upon convergence issues,
     * remove a log file that has no data.
     */
    void pruneEmptyLog();

  protected:
    /**
     * A helper for initializing the file, so that we can get
     * dynamic binding to functions in the classes which inherit from this.
     */
    void initLogFile();

    /**
     * An setup function, to be overridden by childen if desired.
     * Called by setup. Virtual so that derived classes can override.
     */
    virtual void setupHelper();

    // Function for header for this logger.
    // must be defined by subclasses.
    // May need knowledge of the World (example, how many vertices -> number of
    // columns.)
    virtual std::string getLogHeader() = 0;

    // Function for data to be returned by this logger.
    // must be defined by subclasses.
    // Pass in the whole World, so the children can decide what
    // they're interested in recording.
    virtual std::string getLogData() = 0;

    // helper for detecting empty log files (and pruning them).
    // one way is to count the number of lines in the file and match it against
    // the number of lines in the header
    int countLinesInLog();
    int num_lines_header;

    // Full path to the logged file, with prefix prepended and
    // timestap appended.
    std::string m_fileName;

    // Helper to get a timestamp string.
    std::string getTimestamp();
    // Another helper for generating the folder path for the log file
    // Example: May 7th, 2020, 2:51pm and 34 seconds will return:
    // "2020/2020_05/2020_05_07/2020_05_07_2"
    // and the final filename will have the minute and seconds attached.
    std::string getTimeDateFolderPath();

    // A file stream reference. Here, empty constructor, will be replaced in
    // constructor.
    std::ofstream& m_dataFile;

    // and an update period. This many samples in between writes to log file.
    int period;
};

#endif  // BASE_LOGGER_H
