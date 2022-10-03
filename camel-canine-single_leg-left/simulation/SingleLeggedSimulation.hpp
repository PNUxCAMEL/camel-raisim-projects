//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_SIMPLEPENDULUMSIMULATION_HPP
#define RAISIM_SIMPLEPENDULUMSIMULATION_HPP

#include <cmath>
#include <QApplication>
#include "camel-tools/ThreadGenerator.hpp"
#include "Simulation.hpp"
#include "SingleLeggedRobot.hpp"
#include "SingleLeggedSharedMemory.hpp"
#include "GUI/simulationMainwindow.h"
#include "controller/SingleLeggedPDController.hpp"
#include "controller/SingleLeggedIDController.hpp"
#include "controller/SingleLeggedMPCController.hpp"

class SingleLeggedSimulation : public Simulation
{

public:
    SingleLeggedSimulation(raisim::World* world, double dT)
        : Simulation(world, dT)
    {
        ;
    }

private:

};


#endif //RAISIM_SIMPLEPENDULUMSIMULATION_HPP
