#ifndef OPEN_LOOP_UNIFORM_KAPPABAR_CONTROLLER_H
#define OPEN_LOOP_UNIFORM_KAPPABAR_CONTROLLER_H

#include "base_controller.h"

class SoftRobots;

class OpenLoopUniformKappaBarController : public BaseController
{

  public:
    OpenLoopUniformKappaBarController(const shared_ptr<SoftRobots>& soft_robots, string file_path);
    ~OpenLoopUniformKappaBarController();

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimeStep(double dt) override;

  private:
    // helper to parse the csv file. Writes data into time_pts and dutys.
    void parseActuationFile(string csv_path);
    void updatePhies();
    // Store the timepoints for desired kappabar (2by1 vector for a rod) values
    // for all rods. We'll assume index alignment between these two
    vector<double> time_pts;
    vector<vector<double>> desired_phies_profile;  //
    vector<double> desired_phi_list;               //
    // The CSV file will have a certain number of lines as a header. We'll need
    // to remove them.
    int csv_header_lines = 1;

    // For debugging, if desired: detect when we've changed to the next
    // timepoint, so we can pause and inspect the gait.
    int prev_time_pt_idx = 0;
};

#endif  // OPEN_LOOP_UNIFORM_KAPPABAR_CONTROLLER_H
