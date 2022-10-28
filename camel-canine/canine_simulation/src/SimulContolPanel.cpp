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
        , mGaitCounter(0)
        , mGaitLength(5)
        , stand(mGaitLength, Vec4<int>(50,50,50,50), Vec4<int>(50,50,50,50), 50)
        , trot(mGaitLength, Vec4<int>(0,25,25,0), Vec4<int>(25,25,25,25), 50)
        , MPCcontrol(mGaitLength)
{
    mTorque.setZero();
}

void SimulControlPanel::ControllerFunction()
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
            break;
        }
        case STATE_PD_CONTROL:
        {
            break;
        }
        case STATE_TROT_REDAY:
        {
            MPCcontrol.InitSwingLegTrajectory();
            sharedMemory->controlState = STATE_TROT_CONTROL;
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
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