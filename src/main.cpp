#include "eigenIncludes.h"
#include "world.h"
#include "logging/worldLogger.h"
#include "simulation_environments/headlessDERSimulationEnvironment.h"
#include "simulation_environments/openglDERSimulationEnvironment.h"
#include "robotDescription.h"
#include "global_const.h"


shared_ptr<world> my_world;
int verbosity;

// Hack: main creates the output file for logging
ofstream logging_output_file;

double openglDERSimulationEnvironment::render_scale = 1.0;
bool openglDERSimulationEnvironment::show_mat_frames = false;


int main(int argc,char *argv[])
{
    shared_ptr<softRobots> soft_robots = make_shared<softRobots>();
    shared_ptr<forceContainer> forces = make_shared<forceContainer>();
    simParams sim_params;  // TODO: make sure this is correct
    shared_ptr<worldLogger> logger = nullptr;

    get_robot_description(argc, argv, soft_robots, forces, logger, sim_params);

    my_world = make_shared<world>(soft_robots, forces, sim_params);

    verbosity = sim_params.debug_verbosity;

    unique_ptr<derSimulationEnvironment> env;
    if (sim_params.render) {
        env = make_unique<openglDERSimulationEnvironment>(my_world, sim_params, logger, argc, argv);
    }
    else {
        env = make_unique<headlessDERSimulationEnvironment>(my_world, sim_params, logger);
    }

    env->runSimulation();

    exit(0);
}

