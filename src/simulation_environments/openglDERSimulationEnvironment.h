#ifndef OPENGL_DER_SIMULATION_ENVIRONMENT_H
#define OPENGL_DER_SIMULATION_ENVIRONMENT_H

#include "derSimulationEnvironment.h"

// handler for keyboard interrupts in the OpenGL window. No need to be object-oriented here.
extern "C" void keyHandler(unsigned char key, int x, int y);

// Since GLUT is in C, we need global pointers to the world and logger.
// These should be assigned in constructor.
static shared_ptr<world> opengl_world = nullptr;
static shared_ptr<worldLogger> opengl_logger = nullptr;
// and a global for logging / not logging. This is ugly, to-do: fix.
static bool opengl_is_logging = false;
// same with other stuff that needs to be in C
static int opengl_cmdline_per = 0;

class openglDERSimulationEnvironment : public derSimulationEnvironment
{
    public:
    openglDERSimulationEnvironment(const shared_ptr<world>& m_world, const simParams& sim_params,
                                   const shared_ptr<worldLogger>& logger, int argc, char **argv);
    ~openglDERSimulationEnvironment() override;

    /**
     * Setup function, called SEPARATELY.
     */
    // void setup();

    /**
     * Start the simulation!
     */
    void runSimulation();
    static bool show_mat_frames;
    static double render_scale;

    protected:

    // helper that initializes the OpenGL window
    void initGL();

    // OpenGL calls this function to timestep the simulation and update the window.
    // GLUT is in C. A static method works here: uses openglWorld_p and openglWorldLogger_p.
    // What this MEANS is that if multiple instances of this environment are created, they'll all
    // operate on the MOST RECENT world that was created. Ugly, but won't cause segfaults at least.
    // Just don't do multiple GLUT displays at the same time!!!
    static void derOpenGLDisplay();

    // Initializing GLUT requires the command-line arguments from main.
    int argc_main;
    char **argv_main;

};

#endif // OPENGL_DER_SIMULATION_ENVIRONMENT_H
