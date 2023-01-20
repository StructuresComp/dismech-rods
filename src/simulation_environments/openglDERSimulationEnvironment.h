/**
 * openglDERSimulationEnvironment.h
 *
 * Declarations for the concrete class openglDERSimulationEnvironment.
 * Creates, updates, manages, etc., a graphical interface to the DER simulation.
 * Uses GLUT, code originally from main.cpp from Huang et al.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef OPENGL_DER_SIMULATION_ENVIRONMENT_H
#define OPENGL_DER_SIMULATION_ENVIRONMENT_H

// Subclass of simulation environment
#include "derSimulationEnvironment.h"

// // OpenGL calls this function to timestep the simulation and update the window.
// // GLUT is in C so these need to either be static (nope!) or extern "C" methods.
// extern "C" void derOpenGLDisplay();

// handler for keyboard interrupts in the OpenGL window. No need to be object-oriented here.
extern "C" void keyHandler(unsigned char key, int x, int y);

// Since GLUT is in C, we need global pointers to the world and logger.
// These should be assigned in constructor.
static shared_ptr<world> openglWorld_p = nullptr;
static shared_ptr<worldLogger> openglWorldLogger_p = nullptr;
// and a global for logging / not logging. This is ugly, to-do: fix.
static bool opengl_is_logging = false;
// same with other stuff that needs to be in C
static int opengl_cmdline_per = 0;

class openglDERSimulationEnvironment : public derSimulationEnvironment
{
    public:

    /**
     * Constructors do the following:
     * (1) set local static variables because GLUT
     * (2) set the local argc and argv from main, needed for GLUT init
     * (3) call the parents
     */
    openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, int m_argc, char **m_argv);
    openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, shared_ptr<worldLogger> m_logger, int m_argc, char **m_argv);
    ~openglDERSimulationEnvironment();

    /**
     * Setup function, called SEPARATELY.
     */
    // void setup();

    /**
     * Start the simulation!
     */
    void runSimulation();

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
