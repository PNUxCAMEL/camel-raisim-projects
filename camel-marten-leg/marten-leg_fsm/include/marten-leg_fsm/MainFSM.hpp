//
// Created by jh on 22. 11. 21.
//

#ifndef RAISIM_MAINFSM_HPP
#define RAISIM_MAINFSM_HPP


#include <iostream>
#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>

#include <ControlMain/ControllerState.hpp>
#include <marten-leg_util/CanMotor.hpp>
#include <marten-leg_util/Command.hpp>
#include <marten-leg_util/RobotDescription.hpp>
#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_raisim/RobotVisualization.hpp>

void StartFSM();
void *RTControllerThread(void *arg);
void *NRTCAN(void *arg);
void *NRTCommandThread(void *arg);
void *NRTVisualThread(void *arg);
void clearSharedMemory();


#endif //RAISIM_MAINFSM_HPP
