//
// Created by hs on 22. 9. 27.
//

#ifndef RAISIM_CANINEMAIN_HPP
#define RAISIM_CANINEMAIN_HPP

#include <QApplication>

#include <PDcontroller/JointPDController.hpp>
#include <canine_util/MotorCAN.hpp>
#include <canine_util/Command.hpp>
#include <canine_util/ThreadRT.hpp>
#include <canine_util/ThreadNRT.hpp>
#include <canine_util/ThreadFunction.hpp>
#include <canine_gui/mainwindow.h>
#include <canine_raisim//RobotVisualization.hpp>

pthread_t RTThreadController;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;

ThreadRT threadrt;
ThreadNRT threadnrt;
ThreadFunction threadfunc;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

MotorCAN can("can9");
Command userCommand(&can);
JointPDController userController(&can);

raisim::World world;
std::string urdfPath = "\\home\\hs\\raisimLib\\camel-raisim-projects\\camel-urdf\\camel_single_leg_left\\camel_single_leg.urdf";
raisim::RaisimServer server(&world);
RobotVisualization userVisual(&world, urdfPath, &server);

void *RTControllerThread(void* arg);
void *NRTCommandThread(void* arg);
void *NRTVisualThread(void* arg);
void clearSharedMemory();

#endif //RAISIM_CANINEMAIN_HPP
