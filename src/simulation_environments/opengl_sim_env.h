#ifndef OPENGL_SIM_ENV_H
#define OPENGL_SIM_ENV_H

#include "base_sim_env.h"

// handler for keyboard interrupts in the OpenGL window. No need to be
// object-oriented here.
extern "C" void keyHandler(unsigned char key, int x, int y);

// Since GLUT is in C, we need global pointers to the World and logger.
// These should be assigned in constructor.
static shared_ptr<World> opengl_world = nullptr;
static shared_ptr<BaseLogger> opengl_logger = nullptr;
// and a global for logging / not logging. This is ugly, to-do: fix.
static bool opengl_is_logging = false;
// same with other stuff that needs to be in C
static int opengl_cmdline_per = 0;

class OpenGLSimEnv : public BaseSimEnv
{
  public:
    OpenGLSimEnv(const shared_ptr<World>& m_world, const RenderParams& render_params,
                 const shared_ptr<BaseLogger>& logger, int argc, char** argv);
    ~OpenGLSimEnv() override;

    void runSimulation() override;
    void stepSimulation() override;
    static bool show_mat_frames;
    static double render_scale;

  protected:
    // OpenGL calls this function to timestep the simulation and update the
    // window. GLUT is in C. A static method works here: uses openglWorld_p and
    // openglWorldLogger_p. What this MEANS is that if multiple instances of
    // this environment are created, they'll all operate on the MOST RECENT
    // World that was created. Ugly, but won't cause segfaults at least. Just
    // don't do multiple GLUT displays at the same time!!!
    static void derOpenGLDisplay();

    // Initializing GLUT requires the command-line arguments from main.
    int argc_main;
    char** argv_main;
};

#endif  // OPENGL_SIM_ENV
