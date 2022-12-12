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

#include <canine_util/Command.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

#include <canine_simulation/SimulStateEstimator.hpp>
#include <canine_simulation/SimulControlPanel.hpp>
#include <canine_simulation/SimulVisualizer.hpp>
#include <canine_simulation/SimulCommand.hpp>
#include <canine_simulation/SimulXboxCommand.hpp>
#include <convexMPC/MPCController.hpp>

void StartSimulation();
void* NRTCommandThread(void* arg);
void *RTControllerThreadHigh(void *arg);
void *RTControllerThreadLow(void *arg);
void* RTStateEstimator(void* arg);

#endif //RAISIM_SIMULMAIN_HPP
