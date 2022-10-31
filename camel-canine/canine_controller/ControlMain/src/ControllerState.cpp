//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
    , mGaitCounter(0)
    , mGaitLength(5)
    , stand(mGaitLength, Vec4<int>(100,100,100,100), Vec4<int>(100,100,100,100), 100)
    , trot(mGaitLength, Vec4<int>(0,50,50,0), Vec4<int>(50,50,50,50), 100)
    , MPCcontrol(mGaitLength)
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
            PDcontrol.SetControlInput();
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
            break;
        }
        case STATE_PD_CONTROL:
        {
            break;
        }
        case STATE_WBC_READY:
        {
            WBControl.InitTrajectory();
            sharedMemory->controlState = STATE_WBC_CONTROL;
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            sharedMemory->gaitState = STAND;
            break;
        }
        case STATE_WBC_CONTROL:
        {
            stand.setIterations(mGaitCounter);
            sharedMemory->gaitTable = stand.getGaitTable();
            WBControl.DoWBControl();
            mGaitCounter++;
            break;
        }
        case STATE_TROT_REDAY:
        {
            MPCcontrol.InitSwingLegTrajectory();
            sharedMemory->controlState = STATE_TROT_CONTROL;
            sharedMemory->gaitState = STAND;
            break;
        }
        case STATE_TROT_CONTROL:
        {
            stand.setIterations(mGaitCounter);
            sharedMemory->gaitTable = stand.getGaitTable();
            MPCcontrol.DoControl();
            mGaitCounter++;
            break;
        }
        default:
            break;
    }
}