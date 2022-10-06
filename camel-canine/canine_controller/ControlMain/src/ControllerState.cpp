//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
{
}

void ControllerState::ControllerFunction()
{
    sharedMemory->localTime = mIteration * CONTROL_dT;
    mIteration++;
    switch (sharedMemory->controlState)
    {
        case STATE_CONTROL_STOP:
        {
            break;
        }
        case STATE_READY:
        {
            PDcontrol.setControlInput();
            break;
        }
        case STATE_HOME_READY:
        {
            PDcontrol.InitHomeTrajectory();
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
            PDcontrol.DoHomeControl();
            break;
        }
        case STATE_PD_READY:
        {
            sharedMemory->controlState = STATE_PD_CONTROL;
            break;
        }
        case STATE_PD_CONTROL:
        {
            break;
        }
        case STATE_TROT_REDAY:
        {
            PDcontrol.InitSwingTrajectory();
            sharedMemory->controlState = STATE_TROT_CONTROL;
            break;
        }
        case STATE_TROT_CONTROL:
        {
            PDcontrol.DoPDControl();
            break;
        }
        default:
            break;
    }
}