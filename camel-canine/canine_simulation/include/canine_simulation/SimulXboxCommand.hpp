//
// Created by hs on 22. 11. 14.
//

#ifndef RAISIM_SIMULXBOXCOMMAND_HPP
#define RAISIM_SIMULXBOXCOMMAND_HPP

#include <iostream>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/XboxController.hpp>

class SimulXboxCommand {
public:
    void commandFunction();
private:
    XboxController joystick = XboxController();
};

#endif //RAISIM_SIMULXBOXCOMMAND_HPP
