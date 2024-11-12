#include "global_definitions.h"
#include "logging/base_logger.h"
#include "simulation_environments/headless_sim_env.h"
#include "simulation_environments/opengl_sim_env.h"
#include "world.h"
#ifdef WITH_MAGNUM
#include "simulation_environments/magnum_sim_env.h"
#endif
#include "robot_description.h"

std::shared_ptr<World> my_world;

// Hack: main creates the output file for logging
std::ofstream logging_output_file;

double OpenGLSimEnv::render_scale = 1.0;
bool OpenGLSimEnv::show_mat_frames = false;

int main(int argc, char* argv[]) {
    std::shared_ptr<SoftRobots> soft_robots = std::make_shared<SoftRobots>();
    std::shared_ptr<ForceContainer> forces = std::make_shared<ForceContainer>();
    SimParams sim_params;
    RenderParams render_params;
    std::shared_ptr<BaseLogger> logger = nullptr;

    getRobotDescription(argc, argv, soft_robots, forces, logger, sim_params, render_params);
    soft_robots->setup();

    // create the world for the robot to interact with
    my_world = std::make_shared<World>(soft_robots, forces, sim_params);

    std::unique_ptr<BaseSimEnv> env;
    switch (render_params.renderer) {
        case HEADLESS:
            env = std::make_unique<HeadlessSimEnv>(my_world, render_params, logger);
            break;
        case OPENGL:
            env = std::make_unique<OpenGLSimEnv>(my_world, render_params, logger, argc, argv);
            break;
#ifdef WITH_MAGNUM
        case MAGNUM:
            env =
                std::make_unique<Magnum::MagnumSimEnv>(my_world, render_params, logger, argc, argv);
            break;
#endif
        default:
            throw std::runtime_error("Unknown renderer type provided.");
    }
    // run simulation
    env->runSimulation();

    exit(0);
}
