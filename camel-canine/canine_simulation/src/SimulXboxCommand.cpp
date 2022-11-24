//
// Created by hs on 22. 11. 14.
//

#include <canine_simulation/SimulXboxCommand.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

void SimulXboxCommand::commandFunction()
{
    joystick.joyRead();
    std::cout << joystick.joy_button[0] << std::endl;
}