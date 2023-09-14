/**
 * rodOpenLoopFileController.h
 *
 * Declarations for the concrete class rodOpenLoopFileController.
 * Pipes through the output of PWM blocks, based on a given CSV file of duty cycle timepoints.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H
#define ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H

// everything comes from parent
#include "rodController.h"
#include "../rod_mechanics/elasticRod.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodOpenLoopFileKappabarSetter : public rodController
{

public:
    /**
     * Constructor and destructor call parents, but now also pass in:
     * @param m_filepath path to the .csv file that contains the duty cycle timepoints
     */
    rodOpenLoopFileKappabarSetter(int numAct, std::string m_filepath, vector<shared_ptr<elasticRod>> &m_limbs);
    ~rodOpenLoopFileKappabarSetter();

    /**
     * Calculate the desired control input, u(x).
     * Now open loop so rod unused.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    // std::vector<int> getU(shared_ptr<elasticRod> rod_p);
    vector<shared_ptr<elasticRod>> limbs;

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimestep(double dt);
    
    // int numActuators;

private:
    // helper to parse the csv file. Writes data into time_pts and dutys.
    void parseActuationFile(std::string csv_path);
    void updatePhies();
    // and be able to open the actuation file
    // std::string csv_path;
    // Store the timepoints for desired kappabar (2by1 vector for a rod) values for all rods. We'll assume index alignment between these two
    std::vector<double> time_pts;
    // std::vector<double> desired_phies;
    std::vector<std::vector<double>> desired_phies_profile; //
    std::vector<double> desired_phi_list; //
    // The CSV file will have a certain number of lines as a header. We'll need to remove them.
    int csv_header_lines = 1;

    // For debugging, if desired: detect when we've change to the next timepoint, so we can pause and inspect the gait.
    int prev_time_pt_idx = 0;
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);
};

#endif // ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H