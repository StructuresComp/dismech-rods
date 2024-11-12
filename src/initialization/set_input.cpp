#include "set_input.h"

using namespace std;

setInput::setInput() {

    AddOption("render", "visualization", render);
    AddOption("renderScale", "visualization", render_scale);
    AddOption("showMatFrames", "Flag for visualizing material frames", show_mat_frames);
    AddOption("rodRadius", "Radius of Rod", rodRadius);
    AddOption("youngM", "Young's Modulus", youngM);
    AddOption("Poisson", "Poisson Ratio", Poisson);
    AddOption("deltaTime", "Time Step Length", deltaTime);
    AddOption("tol", "Tolerance of Newton Method", tol);
    AddOption("stol", "Ratio between initial and final error", stol);
    AddOption("maxIter", "Maximum Running Times of Each Stepper", maxIter);
    AddOption("density", "Density of the Rod", density);
    AddOption("viscosity", "Viscous Force after wait time", viscosity);
    AddOption("gVector", "Gravity", gVector);
    AddOption("colLimit", "Limit for collision detection to be put in candidate set", col_limit);
    AddOption("delta", "Distance tolerance for contact", delta);
    AddOption("kScaler", "Constant scaler for contact stiffness", k_scaler);
    AddOption("mu", "Coefficient of friction", mu);
    AddOption("nu", "Slipping tolerance for friction ", nu);
    AddOption("lineSearch", "Flag for enabling line search", line_search);
    AddOption("debugVerbosity", "Flag for enabling informative print statements", verbosity);
    AddOption("cmdlinePer", "Period of printing info to command line", cmdline_per);
    AddOption("floorZ", "Z-coordinate of floor plane", floor_z);
    AddOption("simTime", "Total sim duration", sim_time);
    AddOption("integrationScheme", "Integration scheme to be used for time stepping",
              integration_scheme);
    AddOption("phiCtrlFilePath", "Path to Curvature angles' setpoint profiles (.csv) for all limbs",
              phi_ctrl_filepath);
    AddOption("enable2DSim", "Flag to simulate solely along 2D x-z plane", enable_2d_sim);
    AddOption("enableLogging", "Period of printing info to command line", enable_logging);
    AddOption("logfileBase", "Base directory for saving log files", logfile_base);
    AddOption("loggingPeriod", "Frequency of logging", logging_period);
    AddOption("adaptiveTimeStepping", "Adaptive time stepping flag / number of iters",
              adaptive_time_stepping);
}

setInput::~setInput() {
    ;
}

Option* setInput::GetOption(const std::string& name) {
    if (m_options.find(name) == m_options.end()) {
        cerr << "Option " << name << " does not exist" << std::endl;
    }
    return &(m_options.find(name)->second);
}

bool& setInput::GetBoolOpt(const std::string& name) {
    return GetOption(name)->b;
}

int& setInput::GetIntOpt(const std::string& name) {
    return GetOption(name)->i;
}

double& setInput::GetScalarOpt(const std::string& name) {
    return GetOption(name)->r;
}

Vec3& setInput::GetVecOpt(const std::string& name) {
    return GetOption(name)->v;
}

std::string& setInput::GetStringOpt(const std::string& name) {
    return GetOption(name)->s;
}

int setInput::LoadOptions(const char* filename) {
    ifstream input(filename);
    if (!input.is_open()) {
        cerr << "ERROR: File " << filename << " not found" << std::endl;
        return -1;
    }

    std::string line, option;
    istringstream sIn;
    std::string tmp;
    for (getline(input, line); !input.eof(); getline(input, line)) {
        sIn.clear();
        option.clear();
        sIn.str(line);
        sIn >> option;
        if (option.size() == 0 || option.c_str()[0] == '#')
            continue;
        OptionMap::iterator itr;
        itr = m_options.find(option);
        if (itr == m_options.end()) {
            cerr << "Invalid option: " << option << std::endl;
            continue;
        }
        if (itr->second.type == Option::BOOL) {
            sIn >> tmp;
            if (tmp == "true" || tmp == "1")
                itr->second.b = true;
            else if (tmp == "false" || tmp == "0")
                itr->second.b = false;
        }
        else if (itr->second.type == Option::INT) {
            sIn >> itr->second.i;
        }
        else if (itr->second.type == Option::DOUBLE) {
            sIn >> itr->second.r;
        }
        else if (itr->second.type == Option::VEC) {
            Vec3& v = itr->second.v;
            sIn >> v[0];
            sIn >> v[1];
            sIn >> v[2];
        }
        else if (itr->second.type == Option::STRING) {
            sIn >> itr->second.s;
        }
        else {
            cerr << "Invalid option type" << std::endl;
        }
    }
    input.close();

    return 0;
}

int setInput::LoadOptions(int argc, char** argv) {
    std::string option, tmp;
    int start = 0;
    while (start < argc && std::string(argv[start]) != "--")
        ++start;
    for (int i = start + 1; i < argc; ++i) {
        option = argv[i];
        OptionMap::iterator itr;
        itr = m_options.find(option);
        if (itr == m_options.end()) {
            cerr << "Invalid option on command line: " << option << std::endl;
            continue;
        }
        if (i == argc - 1) {
            cerr << "Too few arguments on command line" << std::endl;
            break;
        }
        if (itr->second.type == Option::BOOL) {
            tmp = argv[i + 1];
            ++i;
            if (tmp == "true" || tmp == "1")
                itr->second.b = true;
            if (tmp == "false" || tmp == "0")
                itr->second.b = false;
        }
        else if (itr->second.type == Option::INT) {
            itr->second.i = atoi(argv[i + 1]);
            ++i;
        }
        else if (itr->second.type == Option::DOUBLE) {
            itr->second.r = atof(argv[i + 1]);
            ++i;
        }
        else if (itr->second.type == Option::VEC) {
            if (i >= argc - 3) {
                cerr << "Too few arguments on command line" << std::endl;
                break;
            }
            Vec3& v = itr->second.v;
            v[0] = atof(argv[i + 1]);
            ++i;
            v[1] = atof(argv[i + 1]);
            ++i;
            v[2] = atof(argv[i + 1]);
            ++i;
        }
        else if (itr->second.type == Option::STRING) {
            itr->second.s = argv[i + 1];
            ++i;
        }
        else {
            // cerr << "Invalid option type" << std::endl;
        }
    }
    return 0;
}
