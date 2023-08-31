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
#include "logging/rodNodeLogger.h"
#include "simulation_environments/derSimulationEnvironment.h"
#include "simulation_environments/headlessDERSimulationEnvironment.h"
#include "simulation_environments/openglDERSimulationEnvironment.h"
#include "initialization/setInput.h"
#include "global_const.h"


shared_ptr<world> my_world;
ofstream pull_data;
ofstream node_data;

double time_taken;

bool record_data;
bool record_nodes;

int verbosity;

// Hack: main creates the output file for logging
ofstream logging_output_file;

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

    my_world = make_shared<world>(inputData);
    my_world->setupWorld(argc, argv);

    // Obtain parameters relevant to simulation loop
    verbosity = inputData.GetIntOpt("debugVerbosity");
    int cmdline_per = inputData.GetIntOpt("cmdlinePer");

//    record_data = inputData.GetBoolOpt("saveData");
//    record_nodes = inputData.GetBoolOpt("recordNodes");
//
//    if (record_data) my_world->OpenFile(pull_data, "pull_data");
//    if (record_nodes) my_world->OpenFile(node_data, "node_data");


    unique_ptr<derSimulationEnvironment> env = nullptr;
    bool show_mat_frames = inputData.GetBoolOpt("showMatFrames");

    // Obtain parameters for logging
    shared_ptr<worldLogger> logger = nullptr;
    bool enable_logging = inputData.GetBoolOpt("enableLogging");
    string logfile_base = inputData.GetStringOpt("logfileBase");
    int logging_period = inputData.GetIntOpt("loggingPeriod");

    if (enable_logging) {
        logger = make_shared<rodNodeLogger>("nodes", logfile_base, logging_output_file, my_world, logging_period);

		// This writes the header. You must call it here!
        logger->setup();
    }


    // TODO: will have to add logging versions as well later
    if (my_world->isRender()) {
        if (enable_logging)
            env = make_unique<openglDERSimulationEnvironment>(my_world, cmdline_per, logger, argc, argv, show_mat_frames);
        else
            env = make_unique<openglDERSimulationEnvironment>(my_world, cmdline_per, argc, argv, show_mat_frames);
    }
    else {
        if (enable_logging)
            env = make_unique<headlessDERSimulationEnvironment>(my_world, cmdline_per, logger);
        else
            env = make_unique<headlessDERSimulationEnvironment>(my_world, cmdline_per);
    }

    env->runSimulation();

    // TODO: remove this type of logging
    // Close (if necessary) the data file
//    if (record_data) my_world->CloseFile(pull_data);
//    if (record_nodes) my_world->CloseFile(node_data);
    exit(0);
}

