#ifndef ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H
#define ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H

#include "rodController.h"

class softRobots;

class rodOpenLoopFileKappabarSetter : public rodController
{

public:
    rodOpenLoopFileKappabarSetter(const shared_ptr<softRobots>& soft_robots, string file_path);
    ~rodOpenLoopFileKappabarSetter();

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimestep(double dt) override;

private:
    // helper to parse the csv file. Writes data into time_pts and dutys.
    void parseActuationFile(string csv_path);
    void updatePhies();
    // Store the timepoints for desired kappabar (2by1 vector for a rod) values for all rods. We'll assume index alignment between these two
    vector<double> time_pts;
    vector<vector<double>> desired_phies_profile; //
    vector<double> desired_phi_list; //
    // The CSV file will have a certain number of lines as a header. We'll need to remove them.
    int csv_header_lines = 1;

    // For debugging, if desired: detect when we've change to the next timepoint, so we can pause and inspect the gait.
    int prev_time_pt_idx = 0;
};

#endif // ROD_OPEN_LOOP_FILE_KAPPABAR_SETTER_H