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
        , mGaitLength(1)
        , stand(mGaitLength, Vec4<int>(100,100,100,100), Vec4<int>(100,100,100,100), 100)
        , trot(mGaitLength, Vec4<int>(0,50,50,0), Vec4<int>(50,50,50,50), 100)
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
        case STATE_MPC_REDAY:
        {
            MPCcontrol.InitSwingLegTrajectory();
            sharedMemory->controlState = STATE_MPC_CONTROL;
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            sharedMemory->gaitState = TROT;
            break;
        }
        case STATE_MPC_CONTROL:
        {
            trot.setIterations(mGaitCounter);
            sharedMemory->gaitTable = trot.getGaitTable();
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