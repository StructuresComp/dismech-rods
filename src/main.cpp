/**
 * simDER
 * simDER stands for "[sim]plified [D]iscrete [E]lastic [R]ods"
 * Dec 2017
 * This code is based on previous iterations. 
 * */

//This line is for mac
//#include <GLUT/glut.h>

//This is for linux
#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <ctime>
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
ofstream pull_data;
ofstream node_data;

double time_taken;

bool record_data;
bool record_nodes;

int verbosity;


int main(int argc,char *argv[])
{
    // Load from the options file.
    setInput inputData;
    inputData = setInput();

    if (argc < 2) {
        throw runtime_error("Not enough arguments. Must pass an options file.");
    }

    inputData.LoadOptions(argv[1]);
    inputData.LoadOptions(argc,argv);

    my_world = make_shared<world>(inputData);
    my_world->setRodStepper();

    // Obtain parameters relevant to simulation loop
    verbosity = inputData.GetIntOpt("debugVerbosity");
    int cmdline_per = inputData.GetIntOpt("cmdlinePer");
    bool enable_logging = inputData.GetBoolOpt("enableLogging");

    shared_ptr<worldLogger> logger = nullptr;

    record_data = inputData.GetBoolOpt("saveData");
    record_nodes = inputData.GetBoolOpt("recordNodes");

    if (record_data) my_world->OpenFile(pull_data, "pull_data");
    if (record_nodes) my_world->OpenFile(node_data, "node_data");



    unique_ptr<derSimulationEnvironment> env = nullptr;

    // TODO: will have to add logging versions as well later
    if (my_world->isRender()) {
        env = make_unique<openglDERSimulationEnvironment>(my_world, cmdline_per, argc, argv);
    }
    else {
        env = make_unique<headlessDERSimulationEnvironment>(my_world, cmdline_per);
    }

    env->runSimulation();

    // TODO: remove this type of logging
    // Close (if necessary) the data file
    if (record_data) my_world->CloseFile(pull_data);
    if (record_nodes) my_world->CloseFile(node_data);
    exit(0);
}

