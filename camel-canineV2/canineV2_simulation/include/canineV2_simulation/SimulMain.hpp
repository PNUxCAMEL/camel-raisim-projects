//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULMAIN_HPP
#define RAISIM_SIMULMAIN_HPP

#include <iostream>
#include <QApplication>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <camel-tools/ThreadGenerator.hpp>

#include <canineV2_util/Command.hpp>
#include <canineV2_util/RobotDescription.hpp>
#include <canineV2_util/SharedMemory.hpp>

#include <canineV2_simulation/SimulStateEstimator.hpp>
#include <canineV2_simulation/SimulControlPanel.hpp>
#include <canineV2_simulation/SimulVisualizer.hpp>
#include <canineV2_simulation/SimulCommand.hpp>
#include <canineV2_simulation/SimulXboxCommand.hpp>
#include <canineV2_simulation/SimulKalmanFilter.hpp>
#include <convexMPC/MPCController.hpp>

void StartSimulation();
void* NRTCommandThread(void* arg);
void* NRTXboxCommandThread(void* arg);
void *RTControllerThreadHigh(void *arg);
void *RTControllerThreadLow(void *arg);
void* RTStateEstimator(void* arg);

#endif //RAISIM_SIMULMAIN_HPP
