
#ifndef RAISIM_CANINESIMMAIN_HPP
#define RAISIM_CANINESIMMAIN_HPP

#include <iostream>
#include <QApplication>

#include "camel-tools/ThreadGenerator.hpp"

#include "GUI/operationMainwindow.h"
#include "CanineSimSharedMemory.hpp"
#include "Simulation/CanineSimSimulation.hpp"

void realTimePlot();
void *rt_simulation_thread(void *arg);

extern MainWindow* MainUI;

pthread_t thread_simulation;
pSHM sharedMemory;
raisim::World world;
double dT = 0.005;

camel::raisim_tools::CanineSimSimulation sim = camel::raisim_tools::CanineSimSimulation(&world, dT);

#endif //RAISIM_CANINESIMMAIN_HPP
