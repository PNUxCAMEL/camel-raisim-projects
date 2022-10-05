//
// Created by hs on 22. 9. 27.
//

#ifndef RAISIM_CANINEMAIN_HPP
#define RAISIM_CANINEMAIN_HPP

#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <canine_util/MotorCAN.hpp>
#include <canine_util/Command.hpp>
#include <canine_gui/mainwindow.h>
#include <canine_raisim//RobotVisualization.hpp>

pthread_t RTThreadController;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

//TODO: DIVIDE CAN into 2 for Forward and Backward
MotorCAN can("can5");
Command userCommand(&can);
JointPDController userController(&can);

raisim::World world;
std::string urdfPath = "\\home\\hs\\raisimLib\\camel-raisim-projects\\camel-urdf\\canine\\urdf\\canineV1.urdf";
raisim::RaisimServer server(&world);
RobotVisualization userVisual(&world, urdfPath, &server);

void *RTControllerThread(void* arg);
void *NRTCommandThread(void* arg);
void *NRTVisualThread(void* arg);
void clearSharedMemory();

#endif //RAISIM_CANINEMAIN_HPP
