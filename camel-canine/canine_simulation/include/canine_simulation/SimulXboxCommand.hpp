//
// Created by hs on 22. 11. 14.
//

#ifndef RAISIM_SIMULXBOXCOMMAND_HPP
#define RAISIM_SIMULXBOXCOMMAND_HPP

#include <iostream>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/XboxController.hpp>
#include <canine_util/Filter.hpp>

class SimulXboxCommand {
public:
    SimulXboxCommand();
    void commandFunction();
private:
    CanineFilter::LPF mVxFilter;
    CanineFilter::LPF mVyFilter;
    XboxController joystick = XboxController();
};

#endif //RAISIM_SIMULXBOXCOMMAND_HPP
