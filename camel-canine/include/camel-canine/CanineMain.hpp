//
// Created by hs on 22. 9. 27.
//

#ifndef RAISIM_CANINEMAIN_HPP
#define RAISIM_CANINEMAIN_HPP

#include <QApplication>
#include <canine_gui/mainwindow.h>
#include <canine_util/RealTimeThread.hpp>
#include <convexMPC/qpsolver.hpp>
#include <canine_visualizer/RaisimInit.hpp>

raisim::World world;
std::string urdfPath = "\\home\\hs\\raisimLib\\camel-raisim-projects\\camel-urdf\\canine\\urdf\\canineV1.urdf";
std::string name = "CANINE";

double dT = 0.005;

#endif //RAISIM_CANINEMAIN_HPP
