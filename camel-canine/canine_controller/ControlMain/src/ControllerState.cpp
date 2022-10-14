//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
    , mGaitCounter(0)
    , mGaitLength(1)
    , stand(mGaitLength, Vec4<int>(50,50,50,50), Vec4<int>(50,50,50,50), 50)
    , trot(mGaitLength, Vec4<int>(0,100,100,0), Vec4<int>(100,100,100,100), 200)
    , MPCcontrol(mGaitLength)
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
            PDcontrol.InitSwingTrajectory();
            sharedMemory->controlState = STATE_PD_CONTROL;
            break;
        }
        case STATE_PD_CONTROL:
        {
            trot.setIterations(mGaitCounter);
            sharedMemory->gaitTable = trot.getGaitTable();
            PDcontrol.DoPDControl();
            mGaitCounter++;
            break;
        }
        case STATE_TROT_REDAY:
        {
            sharedMemory->controlState = STATE_TROT_CONTROL;
            break;
        }
        case STATE_TROT_CONTROL:
        {
            stand.setIterations(mGaitCounter);
            sharedMemory->gaitTable = stand.getGaitTable();
            sharedMemory->gaitState = STAND;
            MPCcontrol.DoControl();
            mGaitCounter++;
            break;
        }
        default:
            break;
    }
}