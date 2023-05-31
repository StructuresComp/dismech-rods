/**
 * rodOpenLoopFileController.h
 *
 * Definition(s) for the concrete class rodOpenLoopFileController.
 * Pipes through the output of PWM blocks, based on a given CSV file of duty cycle timepoints.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodOpenLoopFileKappabarSetter.h"
// For exception handling
#include <exception>
// for the CSV file
#include <fstream>

// Constructor and destructor call parents
rodOpenLoopFileKappabarSetter::rodOpenLoopFileKappabarSetter(int numAct, std::string m_filepath, vector<shared_ptr<elasticRod>> &m_limbs) : rodController(numAct), limbs(m_limbs)
{
    // to-do: validate number of columns of file vs. numAct.
    // if (numAct != m_periods.size())
    // {
    //     throw std::invalid_argument("Rod controller received wrong size num of actuators or wrong length lists of periods. Exiting.");
    // }
    // If there's a tilde in the file name prefix, replace with the home directory
    if (m_filepath.at(0) == '~')
    {
        // Get the $HOME environment variable
        std::string home = std::getenv("HOME");
        // Remove the tilde (the first element) from the string
        m_filepath.erase(0, 1);
        // Concatenate the home directory.
        m_filepath = home + m_filepath;
    }
    // Parse the CSV into timepoints.
    parseActuationFile(m_filepath);
    for (int idx = 0; idx<numActuators; idx++){
        desired_phi_list.push_back(0);
    }
}

rodOpenLoopFileKappabarSetter::~rodOpenLoopFileKappabarSetter()
{
}

void rodOpenLoopFileKappabarSetter::parseActuationFile(std::string csv_path)
{
    // open the file and check if it worked
    std::ifstream csv_file(csv_path);
    if (!csv_file.is_open())
    {
        throw std::invalid_argument("Couldn't open the CSV file, check the path.");
    }
    // various helpers for use as we iterate over rows and columns
    std::string temp_row;
    // Read over the first N-many lines of the header. Easiest to iterate on getline...
    for (int i = 0; i < csv_header_lines; i++)
    {
        std::getline(csv_file, temp_row);
        // Debugging
        std::cout << temp_row << std::endl;
    }
    // Then, start reading rows until the end
    while (std::getline(csv_file, temp_row))
    {
        // Parse as a stringstream by commas
        std::stringstream ss_row(temp_row);
        // The first column is assumed to be the timepoint for row i. It's a string at first
        std::string val_ij;
        std::getline(ss_row, val_ij, ',');
        // and we can push it directly back into the timepoints.
        time_pts.push_back(atof(val_ij.c_str()));
        // we'll keep track of the duty cycles for this timepoint in a new vector, which will then be appended to the vector of vectors.
        std::vector<double> phies_list;
        // then add everything else to the duty cycle vector. Boolean check of stringstream is false when emptied out
        while (ss_row)
        {
            // the next column is...
            std::getline(ss_row, val_ij, ',');
            // getline returns an empty string at the end of a line that terminates in a comma.
            if (!val_ij.empty())
            {
                // place it in
                phies_list.push_back(atof(val_ij.c_str()));
            }
        }
        // verify: did this produce the correct number of columns?
        if (phies_list.size() != numActuators)
        {
            throw std::invalid_argument("Error! Your CSV file had an incorrect number of rows in comparison to numAct. Or, you forgot a comma at the end of the line.");
        }
        // if all good, then add to the indexed duty cycle vector
        desired_phies_profile.push_back(phies_list);
    }
}

// override the base class implementation to include PWMs
void rodOpenLoopFileKappabarSetter::updateTimestep(double dt)
{
    // call the base class to update current_time
    rodController::updateTimestep(dt);
    // Find the corresponding timepoint in the list.
    // The actuation file is written by hand so it's probably pretty short, iterating through isn't too inefficient
    int idx = 0;
    // check against both timepoint and fall-off-end-of-list
    while ((idx < time_pts.size()) && (time_pts[idx] <= current_time))
    {
        idx++;
    }
    // the above actually gives us index+1, since we did a bit of manuevering to prevent array out of bounds.
    idx--;
    // Timepoint check during the gait: are we at the next row of the actuation file?
    if (idx > prev_time_pt_idx)
    {
        std::cout << "TIMEPOINT: ROW " << idx << std::endl;
        prev_time_pt_idx++;
    }
    // 
    for (std::size_t i = 0; i < numActuators; i++)
    {
        desired_phi_list.at(i) = (desired_phies_profile.at(idx)).at(i);
    }
    updatePhies();
}

// Implementation of the controller.
void rodOpenLoopFileKappabarSetter::updatePhies()
{
    // Result should be same length as number of actuators
    for (std::size_t limb_idx = 0; limb_idx < numActuators; limb_idx++)
    {
        double phi_value = 0;
        phi_value = desired_phi_list[limb_idx];
        limbs[limb_idx]->updatePhi(phi_value);
    }
}

std::vector<int> rodOpenLoopFileKappabarSetter::getU(shared_ptr<elasticRod> rod_p)
{
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    return u;
}