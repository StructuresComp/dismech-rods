#include "opengl_sim_env.h"
#include "logging/base_logger.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include "world.h"
#include <GL/freeglut.h>

// the callbacks for openGL.
// Note this is a C function now
extern "C" void keyHandler(unsigned char key, int x, int y) {
    switch (key)  // ESCAPE to quit
    {
        case 27:
            exit(0);
    }
}

OpenGLSimEnv::OpenGLSimEnv(const std::shared_ptr<World>& m_world, const RenderParams& render_params,
                           const std::shared_ptr<BaseLogger>& logger, int argc, char** argv)
    : BaseSimEnv(m_world, render_params, logger), argc_main(argc), argv_main(argv) {

    opengl_world = m_world;
    opengl_cmdline_per = render_params.cmd_line_per;
    render_scale = render_params.render_scale;
    show_mat_frames = render_params.show_mat_frames;
    opengl_is_logging = logger != nullptr;
    opengl_logger = logger;

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
    glClearColor(0.7f, 0.7f, 0.7f,
                 0.0f);       // Set background color to black and opaque
    glClearDepth(10.0f);      // Set background depth to farthest
    glShadeModel(GL_SMOOTH);  // Enable smooth shading

    glLoadIdentity();
    gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    glPushMatrix();

    // Other OpenGL setup:
    // keyboard callback. This is a C function
    glutKeyboardFunc(keyHandler);
    // the function that's called by OpenGL to actually run things. This is a
    // static C++ function which can work as a C function in a pinch, as is here
    glutDisplayFunc(derOpenGLDisplay);

    // Here, we take an initial reading for the x0 state.
    // log if specified.
    if (opengl_is_logging) {
        opengl_logger->logWorldData();
    }
}

OpenGLSimEnv::~OpenGLSimEnv() = default;

void OpenGLSimEnv::derOpenGLDisplay(void) {
    // Step the World forward. This takes care of the SMAs, controller, rod,
    // etc., ...
    try {
        opengl_world->updateTimeStep();  // update time step
    }
    catch (std::runtime_error& excep) {
        std::cout << "Caught a std::runtime_error when trying to World->updateTimeStep: "
                  << excep.what() << std::endl;
        std::cout << "Attempting clean shutdown..." << std::endl;
        cleanShutdown(opengl_logger, opengl_is_logging);
        glutLeaveMainLoop();
    }

    // log if specified.
    if (opengl_is_logging) {
        opengl_logger->logWorldData();
    }

    // The helper from our superclass handles command line output.
    cmdlineOutputHelper(opengl_world, opengl_cmdline_per);

    // Clear screen and Z-buffer
    glClear(GL_COLOR_BUFFER_BIT);

    // draw axis
    float axis_len = 1;
    glLineWidth(0.5);

    // Draw axes.
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(axis_len, 0.0, 0.0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, axis_len, 0.0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, axis_len);
    glEnd();

    // much thicker lines
    glLineWidth(4.0);

    // Draw the rod
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 0.0);
    int limb_idx = 0;
    for (const auto& limb : opengl_world->soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i] == 0) {
                glVertex3f(opengl_world->getCoordinate(4 * i, limb_idx) * render_scale,
                           opengl_world->getCoordinate(4 * i + 1, limb_idx) * render_scale,
                           opengl_world->getCoordinate(4 * i + 2, limb_idx) * render_scale);
                glVertex3f(opengl_world->getCoordinate(4 * (i + 1), limb_idx) * render_scale,
                           opengl_world->getCoordinate(4 * (i + 1) + 1, limb_idx) * render_scale,
                           opengl_world->getCoordinate(4 * (i + 1) + 2, limb_idx) * render_scale);
            }
        }
        limb_idx++;
    }

    // Draw joints
    glColor3f(0.0, 0.0, 1.0);
    int n, l;
    for (const auto& joint : opengl_world->soft_robots->joints) {
        for (int i = 0; i < joint->ne; i++) {
            n = joint->connected_nodes[i].first;
            l = joint->connected_nodes[i].second;
            glVertex3f(opengl_world->getCoordinate(4 * n, l) * render_scale,
                       opengl_world->getCoordinate(4 * n + 1, l) * render_scale,
                       opengl_world->getCoordinate(4 * n + 2, l) * render_scale);
            glVertex3f(joint->x(0) * render_scale, joint->x(1) * render_scale,
                       joint->x(2) * render_scale);
        }
    }
    glEnd();

    // Draw material directors
    if (show_mat_frames) {
        glLineWidth(2.5);
        glBegin(GL_LINES);
        limb_idx = 0;
        double x, y, z;
        VecX m1, m2;
        for (const auto& limb : opengl_world->soft_robots->limbs) {
            for (int i = 0; i < limb->ne; i++) {
                if (limb->isEdgeJoint[i] == 0) {
                    x = 0.5 * render_scale *
                        (opengl_world->getCoordinate(4 * i, limb_idx) +
                         opengl_world->getCoordinate(4 * (i + 1), limb_idx));
                    y = 0.5 * render_scale *
                        (opengl_world->getCoordinate(4 * i + 1, limb_idx) +
                         opengl_world->getCoordinate(4 * (i + 1) + 1, limb_idx));
                    z = 0.5 * render_scale *
                        (opengl_world->getCoordinate(4 * i + 2, limb_idx) +
                         opengl_world->getCoordinate(4 * (i + 1) + 2, limb_idx));
                    m1 = 0.05 * opengl_world->getM1(i, limb_idx);
                    m2 = 0.05 * opengl_world->getM2(i, limb_idx);
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
    for (const auto& limb : opengl_world->soft_robots->limbs) {
        for (int i = 0; i < limb->nv; i++) {
            glVertex3f(opengl_world->getCoordinate(4 * i, limb_idx) * render_scale,
                       opengl_world->getCoordinate(4 * i + 1, limb_idx) * render_scale,
                       opengl_world->getCoordinate(4 * i + 2, limb_idx) * render_scale);
        }
        limb_idx++;
    }
    glEnd();
    glFlush();
}

void OpenGLSimEnv::stepSimulation() {
    glutMainLoopEvent();
    glutPostRedisplay();
}

void OpenGLSimEnv::runSimulation() {
    while (opengl_world->simulationRunning()) {
        stepSimulation();
    }
}
