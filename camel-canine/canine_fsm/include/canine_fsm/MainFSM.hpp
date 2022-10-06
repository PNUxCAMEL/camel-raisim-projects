//
// Created by hs on 22. 10. 5.
//

#ifndef RAISIM_MAINFSM_HPP
#define RAISIM_MAINFSM_HPP

#include <iostream>
#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>

#include <ControlMain/ControllerState.hpp>

#include <canine_util/CanMotorForward.hpp>
#include <canine_util/CanMotorBackward.hpp>
#include <canine_util/Command.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_raisim//RobotVisualization.hpp>

void StartFSM();
void *RTControllerThread(void *arg);
void *RTCANForward(void *arg);
void *RTCANBackward(void *arg);
void *NRTCommandThread(void *arg);
void *NRTVisualThread(void *arg);
void clearSharedMemory();

#endif //RAISIM_MAINFSM_HPP
