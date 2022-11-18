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
#include "setInput.h"

world myWorld;
int NPTS;
ofstream pull_data;
ofstream node_data;

clock_t start;
clock_t finish;
double time_taken;

bool record_data;
bool record_nodes;


static void Key(unsigned char key, int x, int y)
{
    switch (key) // ESCAPE to quit
    {
        case 27:
            exit(0);
    }
}


/* Initialize OpenGL Graphics */
void initGL() 
{
    glClearColor(0.7f, 0.7f, 0.7f, 0.0f);  // Set background color to black and opaque
    glClearDepth(10.0f);                   // Set background depth to farthest
    glShadeModel(GL_SMOOTH);               // Enable smooth shading

    glLoadIdentity();
    gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    glPushMatrix();
}

void simulate() {
    start = std::clock();
    myWorld.updateTimeStep(); // update time step
    finish = std::clock();

    if (myWorld.pulling())
    {
        if (record_data) {
            // Record contact data
            if (myWorld.CoutDataC(pull_data)) {
                // Record time taken to complete one time step
                time_taken = double(finish - start) / double(CLOCKS_PER_SEC);
                pull_data << time_taken << endl;
            }
        }

        if (record_nodes) myWorld.outputNodeCoordinates(node_data);
    }
}

void display(void)
{
    double currentTime  = 0;
    while ( myWorld.simulationRunning() > 0)
    {
        //  Clear screen and Z-buffer
        glClear(GL_COLOR_BUFFER_BIT);

        // draw axis
        double axisLen = 1;
        glLineWidth(0.5);

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

        //draw a line
        glColor3f(0.1, 0.1, 0.1);
        glLineWidth(4.0);

        glBegin(GL_LINES);
        for (int i=0; i < NPTS-1; i++)
        {
            glVertex3f( myWorld.getScaledCoordinate(4*i), myWorld.getScaledCoordinate(4*i+1), myWorld.getScaledCoordinate(4*i+2));
            glVertex3f( myWorld.getScaledCoordinate(4*(i+1)), myWorld.getScaledCoordinate(4*(i+1)+1), myWorld.getScaledCoordinate(4*(i+1)+2));
        }
        glEnd();

        glFlush();

        simulate();
    }
    myWorld.CloseFile(pull_data);
    exit(0);
}

int main(int argc,char *argv[])
{
    setInput inputData;
    inputData = setInput();
    inputData.LoadOptions(argv[1]);
    inputData.LoadOptions(argc,argv);

    //read input parameters from txt file and cmd
    myWorld = world(inputData);
    myWorld.setRodStepper();

    record_data = inputData.GetBoolOpt("saveData");
    record_nodes = inputData.GetBoolOpt("recordNodes");

    if (record_data) myWorld.OpenFile(pull_data, "pull_data");
    if (record_nodes) myWorld.OpenFile(node_data, "node_data");

    if (myWorld.isRender()) // if OpenGL visualization is on
    {
        NPTS = myWorld.numPoints();

        glutInit(&argc,argv);
        glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
        glutInitWindowSize (1000, 1000);
        glutInitWindowPosition (100, 100);
        glutCreateWindow ("simDER");
        initGL();
        glutKeyboardFunc(Key);
        glutDisplayFunc(display);
        glutMainLoop();
    }
    else
    {
        while ( myWorld.simulationRunning() > 0)
        {
            simulate();
        }
    }

    // Close (if necessary) the data file
    if (record_data) myWorld.CloseFile(pull_data);
    if (record_nodes) myWorld.CloseFile(node_data);
    exit(0);
}

