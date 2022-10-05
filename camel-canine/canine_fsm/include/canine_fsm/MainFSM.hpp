//
// Created by hs on 22. 10. 5.
//

#ifndef RAISIM_MAINFSM_HPP
#define RAISIM_MAINFSM_HPP

#include <iostream>
#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <canine_util/MotorCAN.hpp>
#include <canine_util/Command.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_raisim//RobotVisualization.hpp>

void StartFSM();
void *RTControllerThread(void* arg);
void *NRTCommandThread(void* arg);
void *NRTVisualThread(void* arg);
void clearSharedMemory();


#endif //RAISIM_MAINFSM_HPP
