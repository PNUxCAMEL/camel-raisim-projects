//
// Created by hs on 22. 10. 27.
//

#include <marten-leg_simulation/SimulControlPanel.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulControlPanel::SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot)
    : mWorld(world)
    , mRobot(robot)
    , mIteration(0)
    , sumedSquaredPositionError(0)
    , sumedSquaredVelocityError(0)
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
        break;
    }
    case STATE_HOME_STAND_UP_READY:
    {
        updateStates();
        PDcontrol.InitHomeStandUpTrajectory();
        sharedMemory->controlState = STATE_HOME_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
//        mRefMPCIteration = mIteration;
        break;
    }
    case STATE_HOME_STAND_DOWN_READY:
    {
        updateStates();
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
    //TODO: curious
    if (sharedMemory->visualState == STATE_UPDATE_VISUAL)
    {
        integrateSimul();
//        GRFNet.Estimate();
        GRFSMO.Estimate();
//        GRFETO.Estimate();
    }
}

void SimulControlPanel::integrateSimul()
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mTorque[idx + 1] = sharedMemory->motorDesiredTorque[idx];
    }
    mRobot->setGeneralizedForce(mTorque);
    mWorld->integrate();
    updateStates();

    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        for (int idx = 0; idx < BUF_SIZE - 1; idx++)
        {
            sharedMemory->bufMotorPosition[motorIdx][BUF_SIZE - idx - 1] = sharedMemory->bufMotorPosition[motorIdx][BUF_SIZE - idx - 2];
            sharedMemory->bufMotorVelocity[motorIdx][BUF_SIZE - idx - 1] = sharedMemory->bufMotorVelocity[motorIdx][BUF_SIZE - idx - 2];
        }
    }
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        sharedMemory->bufMotorPosition[motorIdx][0] = sharedMemory->motorPosition[motorIdx];
        sharedMemory->bufMotorVelocity[motorIdx][0] = sharedMemory->motorVelocity[motorIdx];
    }
}

void SimulControlPanel::updateStates()
{
    double pastHipVerticalVelocity;
    pastHipVerticalVelocity = sharedMemory->hipVerticalVelocity;
    sharedMemory->hipVerticalPosition = LINK1_LENGTH * cos(mRobot->getGeneralizedCoordinate()[1]) + LINK1_LENGTH * cos(mRobot->getGeneralizedCoordinate()[1] + mRobot->getGeneralizedCoordinate()[2]);
    sharedMemory->motorPosition[0] = mRobot->getGeneralizedCoordinate()[1];
    sharedMemory->motorPosition[1] = mRobot->getGeneralizedCoordinate()[2];
    sharedMemory->hipVerticalVelocity = mRobot->getGeneralizedVelocity()[0];
    sharedMemory->hipVerticalAcceleration = (sharedMemory->hipVerticalVelocity - pastHipVerticalVelocity) / CONTROL_dT;
    sharedMemory->motorVelocity[0] = mRobot->getGeneralizedVelocity()[1];
    sharedMemory->motorVelocity[1] = mRobot->getGeneralizedVelocity()[2];
    sharedMemory->motorTorque[0] = mTorque[1];
    sharedMemory->motorTorque[1] = mTorque[2];

//    sharedMemory->motorTorque[2] = mRobot->getGeneralizedForce()[2];
//    sharedMemory->motorTorque[1] = mRobot->getGeneralizedForce()[2];


    if (&(mRobot->getContacts()[0]) != nullptr)
    {
        sharedMemory->measuredGRF = mRobot->getContacts()[0].getImpulse()[2] / CONTROL_dT;
    }
}