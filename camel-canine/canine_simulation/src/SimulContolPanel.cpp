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
        , stand(mGaitLength, Vec4<int>(100,100,100,100), Vec4<int>(100,100,100,100), 100)
        , trot(mGaitLength, Vec4<int>(0,50,50,0), Vec4<int>(50,50,50,50), 100)
        , test(mGaitLength, Vec4<int>(100,100,50,0), Vec4<int>(100,100,50,50), 100)
        , MPCcontrol(mGaitLength)
{
    PDcontrol.SetPDgain(150.0,2.0);
    mTorque.setZero();
}

void SimulControlPanel::ControllerFunction()
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
        case STATE_WBC_READY:
        {
            WBControl.InitTrajectory();
            sharedMemory->controlState = STATE_WBC_CONTROL;
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            break;
        }
        case STATE_WBC_CONTROL:
        {
            WBControl.DoWBControl();
            break;
        }
        case STATE_MPC_UP_REDAY:
        {
            MPCcontrol.InitUpTrajectory();
            sharedMemory->controlState = STATE_MPC_CONTROL;
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            break;
        }
        case STATE_MPC_DOWN_REDAY:
        {
            MPCcontrol.InitDownTrajectory();
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