//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
    , mGaitLength(3)
    , mSwT(50)
    , stand(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT,mSwT), Vec4<int>(mSwT,mSwT,mSwT,mSwT), mSwT)
    , trot(mGaitLength, Vec4<int>(0,mSwT/2,mSwT/2,0), Vec4<int>(mSwT/2,mSwT/2,mSwT/2,mSwT/2), mSwT)
    , test(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT/2,0), Vec4<int>(mSwT,mSwT,mSwT/2,mSwT/2), mSwT)
    , MPCcontrol(mGaitLength, (double)mSwT/200)
{
}

void ControllerState::ControllerFunction()
{
    sharedMemory->localTime = mIteration * CONTROL_dT;
    mIteration++;
    sharedMemory->gaitIteration++;
    switch (sharedMemory->gaitState)
    {
        case STAND:
        {
            stand.setIterations(sharedMemory->gaitIteration);
            sharedMemory->gaitTable = stand.getGaitTable();
            break;
        }
        case TROT:
        {
            trot.setIterations(sharedMemory->gaitIteration);
            sharedMemory->gaitTable = trot.getGaitTable();
            break;
        }
        case TEST:
        {
            test.setIterations(sharedMemory->gaitIteration);
            sharedMemory->gaitTable = test.getGaitTable();
            break;
        }
        default:
        {
            break;
        }
    }
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
        case STATE_HOME_STAND_UP_READY:
        {
            PDcontrol.InitHomeStandUpTrajectory();
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_STAND_DOWN_READY:
        {
            PDcontrol.InitHomeStandDownTrajectory();
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
            PDcontrol.DoHomeControl();
            break;
        }
        case STATE_MPC_UP_REDAY:
        {
            MPCcontrol.InitUpTrajectory();
            sharedMemory->controlState = STATE_MPC_CONTROL;
            break;
        }
        case STATE_MPC_DOWN_REDAY:
        {
            MPCcontrol.InitDownTrajectory();
            sharedMemory->controlState = STATE_MPC_CONTROL;
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