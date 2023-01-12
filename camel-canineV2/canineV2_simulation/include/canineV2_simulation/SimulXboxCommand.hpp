//
// Created by hs on 22. 11. 14.
//

#ifndef RAISIM_SIMULXBOXCOMMAND_HPP
#define RAISIM_SIMULXBOXCOMMAND_HPP

#include <iostream>

#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/RobotDescription.hpp>
#include <canineV2_util/XboxController.hpp>
#include <canineV2_util/Filter.hpp>

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
