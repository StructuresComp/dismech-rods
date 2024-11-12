#include "open_loop_uniform_kappa_bar_controller.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include <fstream>

OpenLoopUniformKappaBarController::OpenLoopUniformKappaBarController(
    const std::shared_ptr<SoftRobots>& soft_robots, std::string filepath)
    : BaseController(soft_robots->limbs) {
    // to-do: validate number of columns of file vs. numAct.
    // if (numAct != m_periods.size())
    // {
    //     throw std::invalid_argument("Rod controller received wrong size num
    //     of actuators or wrong length lists of periods. Exiting.");
    // }
    // If there's a tilde in the file name prefix, replace with the home
    // directory
    if (filepath.empty())
        return;

    if (filepath.at(0) == '~') {
        // Get the $HOME environment variable
        std::string home = getenv("HOME");
        // Remove the tilde (the first element) from the std::string
        filepath.erase(0, 1);
        // Concatenate the home directory.
        filepath = home + filepath;
    }
    // Parse the CSV into timepoints.
    parseActuationFile(filepath);
    for (int idx = 0; idx < num_actuators * 2; idx++) {
        desired_phi_list.push_back(0);
    }
}

OpenLoopUniformKappaBarController::~OpenLoopUniformKappaBarController() = default;

void OpenLoopUniformKappaBarController::parseActuationFile(std::string csv_path) {
    // open the file and check if it worked
    std::ifstream csv_file(csv_path);
    if (!csv_file.is_open()) {
        throw std::invalid_argument("Couldn't open the CSV file, check the path.");
    }
    // various helpers for use as we iterate over rows and columns
    std::string temp_row;
    // Read over the first N-many lines of the header. Easiest to iterate on
    // getline...
    for (int i = 0; i < csv_header_lines; i++) {
        getline(csv_file, temp_row);
    }
    // Then, start reading rows until the end
    while (getline(csv_file, temp_row)) {
        // Parse as a stringstream by commas
        std::stringstream ss_row(temp_row);
        // The first column is assumed to be the timepoint for row i. It's a
        // std::string at first
        std::string val_ij;
        getline(ss_row, val_ij, ',');
        // and we can push it directly back into the timepoints.
        time_pts.push_back(atof(val_ij.c_str()));
        // we'll keep track of the duty cycles for this timepoint in a new
        // std::vector, which will then be appended to the std::vector of vectors.
        std::vector<double> phies_list;
        // then add everything else to the duty cycle std::vector. Boolean check of
        // stringstream is false when emptied out
        while (ss_row) {
            // the next column is...
            getline(ss_row, val_ij, ',');
            // getline returns an empty std::string at the end of a line that
            // terminates in a comma.
            if (!val_ij.empty()) {
                // place it in
                phies_list.push_back(atof(val_ij.c_str()));
            }
        }
        // verify: did this produce the correct number of columns?
        if (phies_list.size() != num_actuators * 2) {
            throw std::invalid_argument(
                "Error! Your CSV file had an incorrect number of rows in "
                "comparison to numAct. Or, you forgot a comma at the end of "
                "the line.");
        }
        // if all good, then add to the indexed duty cycle std::vector
        desired_phies_profile.push_back(phies_list);
    }
}

// override the base class implementation to include PWMs
void OpenLoopUniformKappaBarController::updateTimeStep(double dt) {
    // call the base class to update current_time
    BaseController::updateTimeStep(dt);
    // Find the corresponding timepoint in the list.
    // The actuation file is written by hand so it's probably pretty short,
    // iterating through isn't too inefficient
    int idx = 0;
    // check against both timepoint and fall-off-end-of-list
    while ((idx < time_pts.size()) && (time_pts[idx] <= current_time)) {
        idx++;
    }
    // the above actually gives us index+1, since we did a bit of maneuvering to
    // prevent array out of bounds.
    idx--;
    // Timepoint check during the gait: are we at the next row of the actuation
    // file?
    if (idx > prev_time_pt_idx) {
        //        std::cout << "TIMEPOINT: ROW " << idx << std::endl;
        prev_time_pt_idx++;
    }
    else {
        return;
    }

    for (std::size_t i = 0; i < num_actuators * 2; i++) {
        desired_phi_list.at(i) = (desired_phies_profile.at(idx)).at(i);
    }
    updatePhies();
}

// Implementation of the controller.
void OpenLoopUniformKappaBarController::updatePhies() {
    int idx1, idx2;
    double angle1, angle2;
    // Result should be same length as number of actuators
    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];
        idx1 = 2 * limb_idx;
        idx2 = 2 * limb_idx + 1;

        angle1 = desired_phi_list[idx1] / limb->ne * (M_PI / 180);
        angle2 = desired_phi_list[idx2] / limb->ne * (M_PI / 180);

        for (int i = 1; i < limb->ne; i++) {
            limb->kappa_bar(i, 0) = 2 * tan(angle1 / 2);
            limb->kappa_bar(i, 1) = 2 * tan(angle2 / 2);
        }
    }
}
