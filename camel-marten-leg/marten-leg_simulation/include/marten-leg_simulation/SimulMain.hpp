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

#include <marten-leg_util/Command.hpp>
#include <marten-leg_util/RobotDescription.hpp>
#include <marten-leg_util/SharedMemory.hpp>

#include <marten-leg_simulation/SimulControlPanel.hpp>
#include <marten-leg_simulation/SimulVisualizer.hpp>
#include <marten-leg_simulation/SimulCommand.hpp>

void StartSimulation();

#endif //RAISIM_SIMULMAIN_HPP
