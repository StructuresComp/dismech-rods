/**
 * rodOpenLoopFileController.h
 * 
 * Declarations for the concrete class rodOpenLoopFileController.
 * Pipes through the output of PWM blocks, based on a given CSV file of duty cycle timepoints.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_OPEN_LOOP_FILE_CONTROLLER_H
#define ROD_OPEN_LOOP_FILE_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// except the PWM blocks, which are particular to this controller
#include "pwmPeripheral.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodOpenLoopFileController : public rodController
{

    public:

    /**
     * Constructor and destructor call parents, but now also pass in:
     * @param m_periods a vector of the periods for all actuators. Must be dimension of numAct.
     * @param m_filepath path to the .csv file that contains the duty cycle timepoints
     */
    rodOpenLoopFileController(int numAct, std::vector<double> m_periods, std::string m_filepath);
    ~rodOpenLoopFileController();

    /**
     * Calculate the desired control input, u(x).
     * Now open loop so rod unused.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimestep(double dt);

    private:

    // helper to parse the csv file. Writes data into time_pts and dutys.
    void parseActuationFile(std::string csv_path);

    // We will need to keep track of a set of PWM peripherals.
    std::vector<shared_ptr<pwmPeripheral>> pwm;
    // and be able to open the actuation file
    // std::string csv_path;
    // Store the timepoints for PWM duty cycles. We'll assume index alignment between these two
    std::vector<double> time_pts;
    std::vector<std::vector<double> > dutys; // at index (whatever), a list of the duty cycles for each PWM.

    // The CSV file will have a certain number of lines as a header. We'll need to remove them.
    int csv_header_lines = 4;

    // For debugging, if desired: detect when we've change to the next timepoint, so we can pause and inspect the gait.
    int prev_time_pt_idx = 0;

};

#endif //ROD_OPEN_LOOP_FILE_CONTROLLER_H