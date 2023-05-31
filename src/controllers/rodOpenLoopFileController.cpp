/**
 * rodOpenLoopFileController.h
 * 
 * Definition(s) for the concrete class rodOpenLoopFileController.
 * Pipes through the output of PWM blocks, based on a given CSV file of duty cycle timepoints.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodOpenLoopFileController.h"
// For exception handling
#include <exception>
// for the CSV file
#include <fstream> 

// Constructor and destructor call parents
rodOpenLoopFileController::rodOpenLoopFileController(int numAct, std::vector<double> m_periods, std::string m_filepath) : rodController(numAct)
{
    // to-do: validate number of columns of file vs. numAct.
    if( numAct != m_periods.size() )
    {
        throw std::invalid_argument("Rod controller received wrong size num of actuators or wrong length lists of periods. Exiting.");
    }
    // If there's a tilde in the file name prefix, replace with the home directory
    if (m_filepath.at(0) == '~') {
        // Get the $HOME environment variable
        std::string home = std::getenv("HOME");
        // Remove the tilde (the first element) from the string
        m_filepath.erase(0,1);
        // Concatenate the home directory.
        m_filepath = home + m_filepath;
    }
    // Parse the CSV into timepoints.
    parseActuationFile(m_filepath);
    // Create the PWMs.
    // Assume initial duty cycles are zero.
    for(std::size_t i=0; i < numAct; i++){
        // create and add to the vector at the same time.
        pwm.push_back(make_shared<pwmPeripheral>(m_periods[i], 0.0));
        // Turn the PWM on, with zero duty cycle.
        pwm[i]->setOnOff(true);
    }
}

rodOpenLoopFileController::~rodOpenLoopFileController()
{
}

void rodOpenLoopFileController::parseActuationFile(std::string csv_path)
{
    // open the file and check if it worked
    std::ifstream csv_file(csv_path);
    if( !csv_file.is_open() ) {
        throw std::invalid_argument("Couldn't open the CSV file, check the path.");
    }
    // various helpers for use as we iterate over rows and columns
    std::string temp_row;
    // Read over the first N-many lines of the header. Easiest to iterate on getline...
    for(int i=0; i < csv_header_lines; i++)
    {
        std::getline(csv_file, temp_row);
        // Debugging
        std::cout << temp_row << std::endl;
    }
    // Then, start reading rows until the end
    while(std::getline(csv_file, temp_row))
    {
        // Parse as a stringstream by commas
        std::stringstream ss_row(temp_row);
        // The first column is assumed to be the timepoint for row i. It's a string at first
        std::string val_ij;
        std::getline(ss_row, val_ij, ',');
        // and we can push it directly back into the timepoints.
        time_pts.push_back(atof(val_ij.c_str()));
        // we'll keep track of the duty cycles for this timepoint in a new vector, which will then be appended to the vector of vectors.
        std::vector<double> dutys_i;
        // then add everything else to the duty cycle vector. Boolean check of stringstream is false when emptied out
        while (ss_row)
        {
            // the next column is...
            std::getline(ss_row, val_ij, ',');
            // getline returns an empty string at the end of a line that terminates in a comma.
            if( !val_ij.empty() )
            {
                // place it in
                dutys_i.push_back(atof(val_ij.c_str()));
            }
        }
        // verify: did this produce the correct number of columns?
        if (dutys_i.size() != numActuators)
        {
            throw std::invalid_argument("Error! Your CSV file had an incorrect number of rows in comparison to numAct. Or, you forgot a comma at the end of the line.");
        }
        // if all good, then add to the indexed duty cycle vector
        dutys.push_back(dutys_i);        
    }
}

// override the base class implementation to include PWMs
void rodOpenLoopFileController::updateTimestep(double dt)
{
    // call the base class to update current_time
    rodController::updateTimestep(dt);
    // Find the corresponding timepoint in the list.
    // The actuation file is written by hand so it's probably pretty short, iterating through isn't too inefficient
    int idx = 0;
    // check against both timepoint and fall-off-end-of-list
    while( (time_pts[idx] <= current_time) && (idx < time_pts.size()) )
    {
        idx++;
    }
    // the above actually gives us index+1, since we did a bit of manuevering to prevent array out of bounds.
    idx--;
    // Timepoint check during the gait: are we at the next row of the actuation file?
    if( idx > prev_time_pt_idx)
    {
        std::cout << "TIMEPOINT: ROW " << idx << std::endl;
        prev_time_pt_idx++;
    }
    // assign the corresponding duty cycles.
    for(std::size_t i=0; i < pwm.size(); i++){
        // pwm[i]->setDutyCycle(dutys[idx][i]);
        // Error: we need a row at time 0.0 to specify the initial duty cycles. Presumably all zeros.
        try
        {
            pwm[i]->setDutyCycle((dutys.at(idx)).at(i));
        }
        catch(const std::out_of_range& e)
        {
            std::cerr << "Caught a " << e.what() << " in rodOpenLoopFileController. Did you forget a duty cycles line at time 0.0?" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    // now, time increment the PWMs also.
    // note this should always happen even if the PWMs are off.
    for(auto p_i : pwm){
        p_i->updateTimestep(dt);
    }
}

// Implementation of the controller.
std::vector<int> rodOpenLoopFileController::getU(shared_ptr<elasticRod> rod_p)
{
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    // Just check if everyone is on or off
    for (size_t i = 0; i < pwm.size(); i++)
    {
        u[i] = pwm[i]->getHighLow();
    }
    // Debugging: center of mass of the rod to the terminal.
    // Wuzhou and Zhewei: this will help you get started!
    // std::cout << rod_p->getCOM() << std::endl;
    
    return u;
}