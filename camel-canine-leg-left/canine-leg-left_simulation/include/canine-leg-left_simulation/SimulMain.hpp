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

#include <canine-leg-left_util/Command.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>
#include <canine-leg-left_util/SharedMemory.hpp>

#include <canine-leg-left_simulation/SimulControlPanel.hpp>
#include <canine-leg-left_simulation/SimulVisualizer.hpp>
#include <canine-leg-left_simulation/SimulCommand.hpp>

void StartSimulation();

#endif //RAISIM_SIMULMAIN_HPP
