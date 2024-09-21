#ifndef SETINPUT_H
#define SETINPUT_H

#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

#include "Option.h"
#include "../globalDefinitions.h"

class setInput {
public:

    typedef std::map<std::string, Option> OptionMap;
    OptionMap m_options;

    setInput();

    ~setInput();

    template<typename T>
    int AddOption(const std::string &name, const std::string &desc, const T &def);

    Option *GetOption(const std::string &name);

    bool &GetBoolOpt(const std::string &name);

    int &GetIntOpt(const std::string &name);

    double &GetScalarOpt(const std::string &name);

    Vector3d &GetVecOpt(const std::string &name);

    string &GetStringOpt(const std::string &name);

    int LoadOptions(const char *filename);

    int LoadOptions(const std::string &filename) {
        return LoadOptions(filename.c_str());
    }

    int LoadOptions(int argc, char **argv);

private:
    double rodRadius;
    double youngM;
    double Poisson;
    double shearM;
    double deltaTime;
    double tol, stol;
    int maxIter; // maximum number of iterations
    double density;
    Vector3d gVector;
    double viscosity;
    bool render;
    double render_scale;
    bool show_mat_frames;
    double col_limit;
    double delta;
    double k_scaler;
    double mu;
    double nu;
    bool line_search;
    int verbosity;
    int cmdline_per;
    double floor_z;
    double sim_time;
    string integration_scheme;
    string phi_ctrl_filepath;
    bool enable_2d_sim;
    bool enable_logging;
    string logfile_base;
    int logging_period;
    int adaptive_time_stepping;

};

#include "setInput.tcc"

#endif // PROBLEMBASE_H
