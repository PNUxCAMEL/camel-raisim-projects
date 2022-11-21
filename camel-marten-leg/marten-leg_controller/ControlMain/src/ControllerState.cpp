//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
{
    mTorque.setZero();
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
        break;
    }
    case STATE_HOME_STAND_UP_READY:
    {
        PDcontrol.InitHomeStandUpTrajectory();
        sharedMemory->controlState = STATE_HOME_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;


        break;
    }
    case STATE_HOME_STAND_DOWN_READY:
    {
        PDcontrol.InitHomeStandDownTrajectory();
        sharedMemory->controlState = STATE_HOME_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
        break;
    }
    case STATE_HOME_CONTROL:
    {
        PDcontrol.DoHomeControl();
        break;
    }
    case STATE_PD_READY:
    {
        PDcontrol.InitSineTrajectory();
        sharedMemory->controlState = STATE_PD_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
        break;
    }
    case STATE_PD_CONTROL:
    {
        PDcontrol.DoSineControl();
        break;
    }
    case STATE_MPC_READY:
    {
        MPCcontrol.InitSineTrajectory();
        sharedMemory->controlState = STATE_MPC_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
        break;
    }
    case STATE_MPC_CONTROL:
    {
        MPCcontrol.DoControl();
        break;
    }
    default:
        break;
    }
}