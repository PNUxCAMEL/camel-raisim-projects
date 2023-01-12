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
#include <canineV2_util/CanMotorFL.hpp>
#include <canineV2_util/CanMotorFR.hpp>
#include <canineV2_util/CanMotorRL.hpp>
#include <canineV2_util/CanMotorRR.hpp>
#include <canineV2_util/Command.hpp>
#include <canineV2_util/XboxCommand.hpp>
//#include <canineV2_util/ImuBase.hpp>
#include <canineV2_util/RobotDescription.hpp>
#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/StateEstimator.hpp>
#include <canineV2_util/RobotMath.hpp>
#include <canineV2_raisim//RobotVisualization.hpp>
#include <convexMPC/MPCController.hpp>

void StartFSM();
void clearSharedMemory();
void* RTControllerThreadHigh(void *arg);
void* RTControllerThreadLow(void *arg);
void* NRTCommandThread(void *arg);
void* NRTVisualThread(void *arg);
void* NRTCANFL(void* arg);
void* NRTCANFR(void* arg);
void* NRTCANRL(void* arg);
void* NRTCANRR(void* arg);


#endif //RAISIM_MAINFSM_HPP
