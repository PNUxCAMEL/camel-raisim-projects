//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState()
    : mIteration(0)
    , mGaitLength(MPC_HORIZON)
    , mSwT(GAIT_PERIOD*200)
    , stand(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT,mSwT), Vec4<int>(mSwT,mSwT,mSwT,mSwT), mSwT)
    , trot(mGaitLength, Vec4<int>(0,mSwT/2,mSwT/2,0), Vec4<int>(mSwT/2,mSwT/2,mSwT/2,mSwT/2), mSwT)
    , test(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT/2,0), Vec4<int>(mSwT,mSwT,mSwT/2,mSwT/2), mSwT)
{
}

void ControllerState::ControllerFunction()
{
    sharedMemory->localTime = mIteration * LOW_CONTROL_dT;
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
    switch (sharedMemory->LowControlState)
    {
        case STATE_LOW_CONTROL_STOP:
        {
            break;
        }
        case STATE_LOW_CONTROL_START:
        {
            LowController.DoControl();
            break;
        }
        default:
            break;
    }
}