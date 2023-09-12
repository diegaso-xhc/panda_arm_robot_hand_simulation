/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
//#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include "tool_kits.h"
#include <Eigen/Dense>
#define PI 3.14159265358979323846
using namespace std;
using Eigen::MatrixXd;


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context




// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

char filename[] = "/home/diegoubuntu/Documents/Ab_2022/Simulation/mujoco-2.3.0/Projects/panda_RH8D_R/franka_sim/Franka_panda_RH8D_R.xml";

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void controlLaw(const mjModel* m, mjData* d)
{
    //any++;
    int n_joints = m->nq;
    double ref[n_joints] = {0, // Panda DOF 1
                      0, // Panda DOF 2
                      0.00, // Panda DOF 3
                      0.0, // Panda DOF 4
                      0.0, // Panda DOF 5
                      0, // Panda DOF 6
                      0, // Panda DOF 7
                      0, // Forearm --> Should be kept in 0 as it is not actuated
                      0, // Wrist adduction/abduction
                      0, // Wrist Flexion/Extension
                      0, // Thumb MCP Adduction
                      0, // Thumb MCP Flexion
                      0, // Thumb PIP Flexion
                      0, // Thumn DIP Flexion
                      PI/2, // Index MCP Flexion
                      0, // Index PIP Flexion
                      0, // Index DIP Flexion
                      0, // Middle MCP Flexion
                      0, // MIddle PIP Flexion
                      0, // MIddle DIP Flexion
                      0, // Ring MCP flexion
                      0, // Ring PIP Flexion
                      0, // Ring DIP Flexion
                      0, // Little MCP Flexion
                      0, // Little PIP Flexion
                      0}; // Little DIP Flexion

    float kp = 0.008f;
    double err = 0.0;
    for(int i = 0; i < n_joints; i++)
    {
        err = ref[i] - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp*(err);
    }
}

void controlSystem(const mjModel* m, mjData* d, double ref[])
{
    int n_joints = m->nq;
    float kp = 0.05f;
    double err = 0.0;
    for(int i = 0; i < n_joints; i++)
    {
        err = ref[i] - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp*(err);
    }
}



// main function
int main(int argc, const char** argv)
{    
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        //return 0;
    }

    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";


    if(argc < 2) {m = mj_loadXML(filename, 0, error, 1000);}
    else
    {
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1400, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    double arr_view[] = {86.68,-9.65863,2.54165,0.171193,0.0378619,0.61732};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    //cout << "About to plot your results" << endl;
    Log("Checking linking works properly");

    int n_joints = 26;
    for(int i = 0; i<n_joints; i++)
        d->qpos[i] = 0;

    double ref[n_joints] = {0, // Panda DOF 1
                      PI/4, // Panda DOF 2
                      0.00, // Panda DOF 3
                      0.0, // Panda DOF 4
                      0.0, // Panda DOF 5
                      0, // Panda DOF 6
                      0, // Panda DOF 7
                      0, // Forearm --> Should be kept in 0 as it is not actuated
                      0, // Wrist adduction/abduction
                      0, // Wrist Flexion/Extension
                      0, // Thumb MCP Adduction
                      0, // Thumb MCP Flexion
                      0, // Thumb PIP Flexion
                      0, // Thumn DIP Flexion
                      0, // Index MCP Flexion
                      0, // Index PIP Flexion
                      0, // Index DIP Flexion
                      0, // Middle MCP Flexion
                      PI/4, // MIddle PIP Flexion
                      0, // MIddle DIP Flexion
                      0, // Ring MCP flexion
                      0, // Ring PIP Flexion
                      0, // Ring DIP Flexion
                      0, // Little MCP Flexion
                      0, // Little PIP Flexion
                      0}; // Little DIP Flexion

    float kp = 0.05f;
    double err = 0.0;
    // run main loop, target real-time simulation and 60 fps rendering
    /*
    MatrixXd m2(2,2);
    m2(0,0) = 3;
    m2(1,0) = 2.5;
    m2(0,1) = -1;
    m2(1,1) = 4;
    std::cout << m2 << std::endl;*/
    std::cout << "Update bemerkt" << std::endl;


    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    //mjcb_control = controlLaw;              // initialize a control law

    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);
/*
        for(int i = 0; i < n_joints; i++)
        {
            err = ref[i] - d->qpos[i];
            d->ctrl[i] = d->ctrl[i] + kp*(err);
            if (i == 6)
                std::cout << d->ctrl[i]  << ", " << err << "," << ref[i] << ", " << d->qpos[i] << std::endl;
        }
            //d->ctrl[i] = kp*(ref[i] - d->qpos[corr_sys[i]]);
*/
        /////////////////////////

        controlSystem(m, d, ref);

        //Log(d->qpos[6]);

        //Log(d->time);



        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);


        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
    //delete[] corr_sys;
    return 1;
}
