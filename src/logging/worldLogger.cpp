/**
 * worldLogger.cpp
 *
 * Some minimal definitions for functions in the abstract
 * class worldLogger, mostly related to setup.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "worldLogger.h"

// The C++ Standard Library
#include "world.h"
#include <stdexcept>
#include <time.h> // for the file name of the log file
// for the folder creation
namespace fs = std::filesystem;

// Constructor: creates the file name, stores locals
worldLogger::worldLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, int per) :
                         m_dataFile(df), period(per), num_lines_header(0)
{
    // A quick check on the passed-in strings: must not be the empty string.
    if (fileNamePrefix == "") {
        throw std::invalid_argument("Must specify a prefix for the worldLogger file name!");
    }
    if (logfile_base == "") {
        throw std::invalid_argument("Must specify the folder to be used for logging via logfile-base in options.txt!");
    }

    // We want to be able to expand the "~" string in logfile_base
    // to be the full home directory of the current user.
    // It's a real pain to have to specify the complete directory structure
    // for these log file names.
    // Check if the first character of the string is a tilde:
    if (logfile_base.at(0) == '~') {
        // Get the $HOME environment variable
        std::string home = std::getenv("HOME");
        // Remove the tilde (the first element) from the string
        logfile_base.erase(0,1);
        // Concatenate the home directory.
        logfile_base = home + logfile_base;
    }

    // NOTE: Don't use subdirectories for now
    logfile_base += "/";
//    // Expand out the folder path based on a nicely organized year, month, day, hour heirarchy
//    logfile_base = logfile_base + "/" + getTimeDateFolderPath();
//    if( verbosity >= 1 ){
//        std::cout << "Logging data in the folder " << logfile_base << std::endl;
//    }
//    // Create this folder if it does not already exist.
//    fs::create_directories(logfile_base);

    // Save the file name here.
    // Assume that we'll log to a file in the datafiles directory.
    std::ostringstream sFileName;
    // with the timestamp included.
    sFileName << logfile_base << fileNamePrefix << "_" << getTimestamp();
    //  << ".csv";
    // save to a string, without the csv ending, for just a moment
    m_fileName = sFileName.str();
    // Finally, since it's possible that we will run multiple simulations within one second,
    // check if a previous iteration already created this file and append a 1, 2, 3, etc. on it.
    if( fs::exists(m_fileName + ".csv") ){
        // find the next biggest counter to add
        int repeated_file_num = 2;
        while( fs::exists(m_fileName + "_" + to_string(repeated_file_num) + ".csv") ){
            repeated_file_num++;
        }
        // and tack it on
        m_fileName = m_fileName + "_" + to_string(repeated_file_num);
    }
    // and put the .csv back on. Done!
    m_fileName = m_fileName + ".csv";

    // SOMEONE ELSE must init the log file, since that depends on the derived class!
    // initLogFile(p_world, fileNamePrefix);
}

// destructor: nothing.
worldLogger::~worldLogger()
{
}

void worldLogger::setup()
{
    // Just a wrapper for some additional function calls
    // auxiliary setup first before initializing the log file.
    setupHelper();
    initLogFile();
}

void worldLogger::setupHelper()
{
    // nothing. Children can override.
}

// the helper, to get around C++'s polymorphism implementation
void worldLogger::initLogFile()
{
    // Create the file, and write the header.
    m_dataFile.open(m_fileName.c_str());
    if (!m_dataFile.is_open()) {
        throw std::runtime_error("Log file could not be opened.");
    }
//    // Timestamp the file, TO-DO: including milliseconds, just in case we have multiple starts within a second.
//    m_dataFile << "Log started on " << getTimestamp() << std::endl;
//    // here's where the subclass' implementation is called.
//    m_dataFile << getLogHeader() << std::endl;

    // Close for now. Will get re-opened during each write.
    m_dataFile.close();
    // For checking if empty log, record the number of lines we wrote for just the header
    num_lines_header = countLinesInLog();
}

// counting number of lines in a log.
int worldLogger::countLinesInLog()
{
    // open for reading. We need an input stream for getline
    // m_dataFile.open(m_fileName.c_str(), std::ios::in);
    std::ifstream m_dataFile_in(m_fileName.c_str(), std::ios::in);
    std::string next_line;
    int line_count = 0;
    // loop through
    while(std::getline(m_dataFile_in, next_line)){
        line_count++;
    }
    return line_count;
}

// Writing to the file itself.
void worldLogger::logWorldData()
{
    // Only log at the given period.
    if (world_ptr->getTimeStep() % period == 0) {
        // Open the log file for writing, appending and not overwriting.
        m_dataFile.open(m_fileName.c_str(), std::ios::app);
        // append whatever the children would like to output
        m_dataFile << getLogData() << std::endl;
        // and close again.
        m_dataFile.close();
    }
}

// Return a nice timestamp to be used as part of the file name.
std::string worldLogger::getTimestamp()
{
    // Credit to Brian Tietz Mirletz, via the NASA Tensegrity Robotics Toolkit.
    // Adapted from: http://www.cplusplus.com/reference/clibrary/ctime/localtime/
    // Also http://www.cplusplus.com/forum/unices/2259/
    time_t rawtime;
    tm* currentTime;
    int fileTimeSize = 64;
    char fileTime [fileTimeSize];

    time (&rawtime);
    currentTime = localtime(&rawtime);
    // strftime(fileTime, fileTimeSize, "%m%d%Y_%H%M%S", currentTime);
    // Formatting consistent with the new folder structure:
//    strftime(fileTime, fileTimeSize, "%Y_%m_%d_%H%M%S", currentTime);
    strftime(fileTime, fileTimeSize, "%m_%d_%H_%M_%S", currentTime);
    // memory management?
    // delete currentTime;
    // Result: fileTime is a string with the time information.
    return fileTime;
}

// Return a folder path formatted by year, month, day hour for a log.
std::string worldLogger::getTimeDateFolderPath()
{
    // Get the current time
    time_t rawtime;
    tm* currentTime;
    time (&rawtime);
    currentTime = localtime(&rawtime);
    // Build up the result
    std::ostringstream folderpath;
    // Some notes: tm_year is since 1900 so have to add, tm_mon is months since Jan (0-11) so have to +1.
    // year
//    folderpath << currentTime->tm_year + 1900 << "/";
//    // month
//    folderpath << currentTime->tm_year + 1900 << "_" << currentTime->tm_mon + 1 << "/";
//    // day
//    folderpath << currentTime->tm_year + 1900 << "_" << currentTime->tm_mon + 1 << "_" << currentTime->tm_mday << "/";
//    // hour
//    folderpath << currentTime->tm_year + 1900 << "_" << currentTime->tm_mon + 1 << "_" << currentTime->tm_mday << "_" << currentTime->tm_hour << "/";
    // memory management?
    // delete currentTime;

    folderpath << currentTime->tm_year + 1900 << "_" << currentTime->tm_mon + 1 << "_" << currentTime->tm_mday << "_" << currentTime->tm_hour << "/";
    return folderpath.str();
}

// If there is no data in the log file (e.g., if the simulation starts at an invalid state),
// delete the log we created.
void worldLogger::pruneEmptyLog()
{
    // debugging
    if( verbosity >= 1 ){
        std::cout << "Checking if the log file " << m_fileName << " is empty..." << std::endl;
    }
    // TO-DO: pass this in as a parameter!
    int min_useful_samples = 50;
    // if(countLinesInLog() == num_lines_header){
    if( (countLinesInLog() - num_lines_header) < min_useful_samples){
        if( verbosity >= 1 ){
            std::cout << "Log file was (almost) empty, removing..." << std::endl;
        }
        int unsuccessful = remove(m_fileName.c_str());
        if( verbosity >= 1 ){
            if( unsuccessful ){
                std::cout << "Error removing file!" << std::endl;
            }
            else
            {
                std::cout << "File removal successful. Exiting now." << std::endl;
            }

        }
    }
    else{
        if( verbosity >= 1 ){
            std::cout << "Log file contained data, not removing. Exiting now." << std::endl;
        }
    }
}
