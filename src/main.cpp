/**
 * simDER
 * simDER stands for "[sim]plified [D]iscrete [E]lastic [R]ods"
 * Dec 2017
 * This code is based on previous iterations.
 * */

#include <iostream>
#include "eigenIncludes.h"

// Rod and stepper are included in the world
#include "world.h"
#include "logging/worldLogger.h"
#include "simulation_environments/derSimulationEnvironment.h"
#include "simulation_environments/headlessDERSimulationEnvironment.h"
#include "simulation_environments/openglDERSimulationEnvironment.h"
#include "initialization/setInput.h"
#include "global_const.h"


shared_ptr<world> my_world;
int verbosity;

// Hack: main creates the output file for logging
ofstream logging_output_file;

double openglDERSimulationEnvironment::render_scale = 1.0;
bool openglDERSimulationEnvironment::show_mat_frames = false;


int main(int argc,char *argv[])
{
    // Load from the options file.
    setInput inputData;
    inputData = setInput();

    if (argc < 2) {
        throw runtime_error("Not enough arguments. Must pass an options file.");
    }

    inputData.LoadOptions(argv[1]);
    inputData.LoadOptions(argc, argv);

    shared_ptr<worldLogger> logger = nullptr;
    my_world = make_shared<world>(inputData);
    my_world->setupWorld(argc, argv, inputData, logger);

    // Obtain parameters relevant to simulation loop
    verbosity = inputData.GetIntOpt("debugVerbosity");
    int cmdline_per = inputData.GetIntOpt("cmdlinePer");
    double render_scale = inputData.GetScalarOpt("renderScale");
    bool show_mat_frames = inputData.GetBoolOpt("showMatFrames");
    bool enable_logging = inputData.GetBoolOpt("enableLogging");

    unique_ptr<derSimulationEnvironment> env;
    if (my_world->isRender()) {
        if (enable_logging && logger)
            env = make_unique<openglDERSimulationEnvironment>(my_world, cmdline_per, logger, argc, argv,
                                                              render_scale, show_mat_frames);
        else
            env = make_unique<openglDERSimulationEnvironment>(my_world, cmdline_per, argc, argv, render_scale,
                                                              show_mat_frames);
    }
    else {
        if (enable_logging && logger)
            env = make_unique<headlessDERSimulationEnvironment>(my_world, cmdline_per, logger);
        else
            env = make_unique<headlessDERSimulationEnvironment>(my_world, cmdline_per);
    }

    env->runSimulation();

    exit(0);
}

