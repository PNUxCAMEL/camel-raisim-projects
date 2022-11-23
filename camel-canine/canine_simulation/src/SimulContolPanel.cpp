//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulControlPanel.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulControlPanel::SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot)
    : mWorld(world)
    , mRobot(robot)
    , mIteration(0)
    , mGaitLength(3)
    , mSwT(GAIT_PERIOD*1000) //0.25s swing
    , stand(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT,mSwT), Vec4<int>(mSwT,mSwT,mSwT,mSwT), mSwT)
    , trot(mGaitLength, Vec4<int>(0,mSwT/2,mSwT/2,0), Vec4<int>(mSwT/2,mSwT/2,mSwT/2,mSwT/2), mSwT)
    , test(mGaitLength, Vec4<int>(mSwT,mSwT,mSwT/2,0), Vec4<int>(mSwT,mSwT,mSwT/2,mSwT/2), mSwT)
{
    mTorque.setZero();
}

void SimulControlPanel::ControllerFunction()
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
    if (sharedMemory->visualState == STATE_UPDATE_VISUAL)
    {
        integrateSimul();
    }
}

void SimulControlPanel::integrateSimul()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        mTorque[idx+6] = sharedMemory->motorDesiredTorque[idx];
    }
    mRobot->setGeneralizedForce(mTorque);
    mWorld->integrate();
}