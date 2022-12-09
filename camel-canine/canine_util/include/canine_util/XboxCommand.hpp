
#ifndef RAISIM_XBOXCOMMAND_HPP
#define RAISIM_XBOXCOMMAND_HPP

#include <iostream>

#include "SharedMemory.hpp"
#include "RobotDescription.hpp"
#include "XboxController.hpp"
#include "Filter.hpp"

class XboxCommand {
public:
    XboxCommand();
    void commandFunction();
private:
    CanineFilter::LPF mVxFilter;
    CanineFilter::LPF mVyFilter;
    XboxController joystick = XboxController();
};


#endif //RAISIM_XBOXCOMMAND_HPP
