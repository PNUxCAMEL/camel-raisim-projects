//
// Created by hs on 22. 10. 27.
//

#include <canine-leg-left_simulation/SimulControlPanel.hpp>

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
//        IDcontrol.InitQuinticTrajectory();
        MPCcontrol.InitSineTrajectory();
        mRefMPCIteration = mIteration;
        sharedMemory->controlState = STATE_PD_CONTROL;
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
        break;
    }
    case STATE_PD_CONTROL:
    {
//        IDcontrol.DoControl();
        if ((mIteration - mRefMPCIteration - 1) % (int)(MPC_dT / CONTROL_dT) == 0)
        {
            MPCcontrol.DoControl();
        }
        GRFcontrol.DoControl();
//        MPCcontrol.SetControlInput();

        sumedSquaredPositionError += pow(sharedMemory->desiredHipVerticalPosition - sharedMemory->hipVerticalPosition, 2);
        sumedSquaredVelocityError += pow(sharedMemory->desiredHipVerticalVelocity - sharedMemory->hipVerticalVelocity, 2);
        positionRMSE = pow(sumedSquaredPositionError / (mIteration - mRefMPCIteration), 0.5);
        velocityRMSE = pow(sumedSquaredVelocityError / (mIteration - mRefMPCIteration), 0.5);
        std::cout << "RMSE position : " << positionRMSE << std::endl;
        std::cout << "RMSE velocity : " << velocityRMSE << std::endl;
        break;
    }
    case STATE_TROT_REDAY:
    {
        break;
    }
    case STATE_TROT_CONTROL:
    {
        break;
    }
    default:
        break;
    }
    //TODO: curious
    if (sharedMemory->visualState == STATE_UPDATE_VISUAL)
    {
        integrateSimul();
        GRFNet.Estimate();
        GRFSMO.Estimate();
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

    for (int idx = 0; idx < BUF_SIZE - 1; idx++)
    {
        for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
        {
            sharedMemory->bufMotorPosition[motorIdx][idx + 1] = sharedMemory->bufMotorPosition[motorIdx][idx];
            sharedMemory->bufMotorVelocity[motorIdx][idx + 1] = sharedMemory->bufMotorVelocity[motorIdx][idx];
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
    sharedMemory->hipVerticalPosition = mRobot->getGeneralizedCoordinate()[0];
    sharedMemory->motorPosition[0] = mRobot->getGeneralizedCoordinate()[1];
    sharedMemory->motorPosition[1] = mRobot->getGeneralizedCoordinate()[2];
    sharedMemory->hipVerticalVelocity = mRobot->getGeneralizedVelocity()[0];
    sharedMemory->hipVerticalAcceleration = (sharedMemory->hipVerticalVelocity - pastHipVerticalVelocity) / CONTROL_dT;
    sharedMemory->motorVelocity[0] = mRobot->getGeneralizedVelocity()[1];
    sharedMemory->motorVelocity[1] = mRobot->getGeneralizedVelocity()[2];
    if (&(mRobot->getContacts()[0]) != nullptr)
    {
        sharedMemory->measuredGRF = mRobot->getContacts()[0].getImpulse()[2] / CONTROL_dT;
    }
}