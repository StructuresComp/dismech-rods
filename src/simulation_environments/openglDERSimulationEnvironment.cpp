/**
 * openglDERSimulationEnvironment.cpp
 *
 * Definitions for the class openglDERSimulationEnvironment.
 * Creates, updates, manages, etc., a graphical interface to the DER simulation.
 * Uses GLUT, code originally from main.cpp from Huang et al.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

// all includes should be in header
#include "openglDERSimulationEnvironment.h"

// Includes for GLUT.
// We need the one with "ext" to get glutLeaveMainLoop
#include <GL/freeglut.h>

// the callbacks for openGL.
// Note this is a C function now
extern "C" void keyHandler(unsigned char key, int x, int y) {
    switch (key) // ESCAPE to quit
    {
        case 27:
            exit(0);
    }
}

#include <ctime>


// Constructors just call parents and store command-line arguments from main
openglDERSimulationEnvironment::openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per,
                                                               int m_argc, char **m_argv, bool m_show_mat_frames) :
                                                               derSimulationEnvironment(m_world, m_cmdline_per),
                                                               argc_main(m_argc), argv_main(m_argv) {
    // also make static copies of the passed-in pointers.
    openglWorld_p = m_world;
    opengl_is_logging = false;
    opengl_cmdline_per = m_cmdline_per;
    show_mat_frames = m_show_mat_frames;
}

openglDERSimulationEnvironment::openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per,
                                                               shared_ptr<worldLogger> m_logger, int m_argc,
                                                               char **m_argv, bool m_show_mat_frames) :
                                                               derSimulationEnvironment(m_world, m_cmdline_per, m_logger),
                                                               argc_main(m_argc), argv_main(m_argv) {
//                                                               show_mat_frames(m_show_mat_frames) {

    openglWorld_p = m_world;
    openglWorldLogger_p = m_logger;
    opengl_is_logging = true;
    opengl_cmdline_per = m_cmdline_per;
    show_mat_frames = m_show_mat_frames;
}

openglDERSimulationEnvironment::~openglDERSimulationEnvironment() {
}

/* Initialize OpenGL Graphics */
void openglDERSimulationEnvironment::initGL() {
    // Initialize GLUT
    glutInit(&argc_main, argv_main);
    // When the window closes, return control back to main
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    // Create the window.
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(1500, 1200);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("disMech");

    // set window properties
    glClearColor(0.7f, 0.7f, 0.7f, 0.0f); // Set background color to black and opaque
    glClearDepth(10.0f);                   // Set background depth to farthest
    glShadeModel(GL_SMOOTH);   // Enable smooth shading

    glLoadIdentity();
    gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
//	gluLookAt(0.00, 0.00, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glPushMatrix();

    // Other OpenGL setup:
    // keyboard callback. This is a C function
    glutKeyboardFunc(keyHandler);
    // the function that's called by OpenGL to actually run things. This is a static C++ function which can work as a C function in a pinch, as is here
    glutDisplayFunc(derOpenGLDisplay);

    // Here, we take an initial reading for the x0 state.
    // log if specified.
    if (opengl_is_logging) {
        openglWorldLogger_p->logWorldData();
    }
}

void openglDERSimulationEnvironment::derOpenGLDisplay(void) {
    // openglWorld_p is static.
    // world knows its max time from setInput
    clock_t t = clock();
    while (openglWorld_p->simulationRunning() > 0) {
        //  Clear screen and Z-buffer
        glClear(GL_COLOR_BUFFER_BIT);

        // draw axis
        double axisLen = 1;
        glLineWidth(0.5);

        // Draw axes.
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(axisLen, 0.0, 0.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, axisLen, 0.0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, axisLen);
        glEnd();

        // much thicker lines
        glLineWidth(4.0);

        // Draw the rod
        glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 0.0);
        int limb_idx = 0;
        for (const auto &limb: openglWorld_p->limbs) {
            for (int i = 0; i < limb->ne; i++) {
                if (limb->isEdgeJoint[i] == 0) {
                    glVertex3f(openglWorld_p->getScaledCoordinate(4 * i, limb_idx),
                               openglWorld_p->getScaledCoordinate(4 * i + 1, limb_idx),
                               openglWorld_p->getScaledCoordinate(4 * i + 2, limb_idx));
                    glVertex3f(openglWorld_p->getScaledCoordinate(4 * (i + 1), limb_idx),
                               openglWorld_p->getScaledCoordinate(4 * (i + 1) + 1, limb_idx),
                               openglWorld_p->getScaledCoordinate(4 * (i + 1) + 2, limb_idx));
                }
            }
            limb_idx++;
        }

        // Draw joints
        glColor3f(0.0, 0.0, 1.0);
        double scale = 0.2;  // hard coding this for now cuz I'm lazy, will fix later
        int n, l;
        for (const auto &joint: openglWorld_p->joints) {
            for (int i = 0; i < joint->ne; i++) {
                n = joint->connected_nodes[i].first;
                l = joint->connected_nodes[i].second;
                glVertex3f(openglWorld_p->getScaledCoordinate(4 * n, l),
                           openglWorld_p->getScaledCoordinate(4 * n + 1, l),
                           openglWorld_p->getScaledCoordinate(4 * n + 2, l));
                glVertex3f(joint->x(0) / scale,
                           joint->x(1) / scale,
                           joint->x(2) / scale);
            }
        }
        glEnd();

        // Draw material directors
        if (show_mat_frames) {
            glLineWidth(2.5);
            glBegin(GL_LINES);
            limb_idx = 0;
            double x, y, z;
            VectorXd m1, m2;
            for (const auto &limb: openglWorld_p->limbs) {
                for (int i = 0; i < limb->ne; i++) {
                    if (limb->isEdgeJoint[i] == 0) {
                        x = 0.5 * (openglWorld_p->getScaledCoordinate(4 * i, limb_idx) +
                                   openglWorld_p->getScaledCoordinate(4 * (i + 1), limb_idx));
                        y = 0.5 * (openglWorld_p->getScaledCoordinate(4 * i + 1, limb_idx) +
                                   openglWorld_p->getScaledCoordinate(4 * (i + 1) + 1, limb_idx));
                        z = 0.5 * (openglWorld_p->getScaledCoordinate(4 * i + 2, limb_idx) +
                                   openglWorld_p->getScaledCoordinate(4 * (i + 1) + 2, limb_idx));
                        m1 = 0.05 * openglWorld_p->getM1(i, limb_idx);
                        m2 = 0.05 * openglWorld_p->getM2(i, limb_idx);
                        glColor3f(1.0, 0.0, 0.0);
                        glVertex3f(x, y, z);
                        glVertex3f(x + m1[0], y + m1[1], z + m1[2]);
                        glColor3f(0.5, 0.0, 1.0);
                        glVertex3f(x, y, z);
                        glVertex3f(x + m2[0], y + m2[1], z + m2[2]);
                    }
                }
                limb_idx++;
            }
            glEnd();
        }

        // Draw nodes
        glPointSize(4.0);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);
        limb_idx = 0;
        for (const auto &limb: openglWorld_p->limbs) {
            for (int i = 0; i < limb->nv; i++) {
//                if (limb->isNodeJoint[i] == 0) {
                glVertex3f(openglWorld_p->getScaledCoordinate(4 * i, limb_idx),
                           openglWorld_p->getScaledCoordinate(4 * i + 1, limb_idx),
                           openglWorld_p->getScaledCoordinate(4 * i + 2, limb_idx));
//                }
            }
            limb_idx++;
        }
        glEnd();


//		double wallAngle = 6.0 * 3.141592654 / 180.0;
//
//		for (double xb = - axisLen; xb < axisLen; xb += axisLen/100.0)
//		{
//			glVertex3f( xb, openglWorld_p->getScaledBoundary(xb), 0);
//			glVertex3f( xb + axisLen/100.0, openglWorld_p->getScaledBoundary(xb+axisLen/100.0), 0);
//		}

        glFlush();

        // Step the world forward. This takes care of the SMAs, controller, rod, etc., ...
        // openglWorld_p->updateTimeStep();
        // catch convergence errors
        try {
            openglWorld_p->updateTimeStep(); // update time step
        }
        catch (std::runtime_error &excep) {
            if (verbosity >= 1) {
                std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what()
                          << std::endl;
                std::cout << "Attempting clean shutdown..." << std::endl;
            }
            // superclass has the method
            cleanShutdown(openglWorldLogger_p, opengl_is_logging);
            // ugly to return here, but that's life
            // exit(1);
            // return;
            // FreeGLUT has a routine for actually stopping the GUI and returning cleanly
            glutLeaveMainLoop();
        }

        // log if specified.
        if (opengl_is_logging) {
            openglWorldLogger_p->logWorldData();
        }

        // The helper from our superclass handles command line output.
        cmdlineOutputHelper(openglWorld_p, opengl_cmdline_per);

        // sleep(0.1);
    }
    cout << "total time " << (double)(clock() -  t) / CLOCKS_PER_SEC << endl;
    // exit(1);
    // return;
    // FreeGLUT has a routine for actually stopping the GUI and returning cleanly
    glutLeaveMainLoop();
}

// Simply start up openGL.
void openglDERSimulationEnvironment::runSimulation() {
    // let our users know to expect a GUI
    if (verbosity >= 1) {
        std::cout << "Using a graphical simulation environment. Opening an openGL window..." << std::endl;
    }
    // First, setup the openGL window
    initGL();
    // then start up openGL.
    glutMainLoop();
}
