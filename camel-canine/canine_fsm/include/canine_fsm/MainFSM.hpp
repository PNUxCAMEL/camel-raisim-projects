//
// Created by hs on 22. 10. 5.
//

#ifndef RAISIM_MAINFSM_HPP
#define RAISIM_MAINFSM_HPP

#include <iostream>
#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>
#include <camel-tools/sensor.hpp>

#include <ControlMain/ControllerState.hpp>
#include <canine_util/CanMotorForward.hpp>
#include <canine_util/CanMotorBackward.hpp>
#include <canine_util/Command.hpp>
//#include <canine_util/ImuBase.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/StateEstimator.hpp>
#include <canine_raisim//RobotVisualization.hpp>
#include <convexMPC/MPCController.hpp>

void StartFSM();
void clearSharedMemory();
void* RTControllerThreadHigh(void *arg);
void* RTControllerThreadLow(void *arg);
void* NRTCommandThread(void *arg);
void* NRTVisualThread(void *arg);
void* NRTCANForward(void* arg);
void* NRTCANBackward(void* arg);

#endif //RAISIM_MAINFSM_HPP
