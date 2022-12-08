//
// Created by hs on 22. 11. 14.
//

#include <canine_simulation/SimulXboxCommand.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulXboxCommand::SimulXboxCommand()
    : mVxFilter(CMD_dT, 100000)
    , mVyFilter(CMD_dT, 100000)
{
}

void SimulXboxCommand::commandFunction()
{
    joystick.joyRead();

    if (abs(joystick.joy_axis[1]) < 5000)
    {
        sharedMemory->baseDesiredVelocity[0] = 0;
    }
    else
    {
        double temp = abs(joystick.joy_axis[1]) - 5000;
        if (joystick.joy_axis[1] < 0)
        {
            sharedMemory->baseDesiredVelocity[0] = floor(temp*100/27767/10)/100;
        }
        else
        {
            sharedMemory->baseDesiredVelocity[0] = -floor(temp*100/27767/10)/100;
        }
    }
    mVxFilter.GetFilteredVar(sharedMemory->baseDesiredVelocity[0]);

    if (abs(joystick.joy_axis[0]) < 5000)
    {
        sharedMemory->baseDesiredVelocity[1] = 0;
    }
    else
    {
        double temp = abs(joystick.joy_axis[0]) - 5000;
        if (joystick.joy_axis[0] < 0)
        {
            sharedMemory->baseDesiredVelocity[1] = floor(temp*100/27767/10)/100;
        }
        else
        {
            sharedMemory->baseDesiredVelocity[1] = -floor(temp*100/27767/10)/100;
        }
    }
    mVyFilter.GetFilteredVar(sharedMemory->baseDesiredVelocity[1]);
}