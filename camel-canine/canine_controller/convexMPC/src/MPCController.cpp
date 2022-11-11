//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , ConvexMPCSolver(mHorizon)
    , SwingLegTrajectory(0.5)
{
    for(double & motorIdx : mTorqueLimit)
    {
        motorIdx = 15.0;
    }
    mGRF->setZero();
    mTorque.setZero();
    robotJacobian->setZero();
    robottorque->setZero();
    swingtorque->setZero();
}

void MPCController::InitTrajectory()
{
    SwingLegTrajectory.UpdateTrajectory(sharedMemory->localTime);
    double timeDuration = 2.0;
    mBaseTrajectory.updateTrajectory(sharedMemory->basePosition[2], 0.3,
                                        sharedMemory->localTime, timeDuration);
}

void MPCController::DoControl()
{
    //TODO: Make structure for robot states
    updateState();
    ConvexMPCSolver.SetTrajectory(mBaseTrajectory);
    ConvexMPCSolver.GetMetrices(mBaseEulerPosition, mBasePosition,
                                mBaseEulerVelocity, mBaseVelocity,
                                mFootPosition);
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    setLegcontrol();
    computeControlInput();
    setControlInput();
}

void MPCController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseEulerPosition[idx] = sharedMemory->baseEulerPosition[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
    }

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            mFootPosition[leg][mt] = sharedMemory->footPosition[leg][mt];
        }
    }

    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        mMotorPosition[idx] = sharedMemory->motorPosition[idx];
        mMotorVelocity[idx] = sharedMemory->motorVelocity[idx];
    }
}

void MPCController::setLegcontrol()
{
    SwingLegTrajectory.GetPositionTrajectory(sharedMemory->localTime, mDesiredPosition);

    double d = sqrt(pow(mDesiredPosition[0],2)+pow(mDesiredPosition[1],2));
    double phi = acos(abs(mDesiredPosition[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));

    double jointPos[3];
    double jointVel[3] = {0,0,0};

    jointPos[0] = 0.f;
    if (mDesiredPosition[0] < 0)
        jointPos[1] = 1.57 - phi + psi;
    else if(mDesiredPosition[0] == 0)
        jointPos[1] = psi;
    else
        jointPos[1] = phi + psi - 1.57;
    jointPos[2] = -acos((pow(d,2)-2*pow(0.23,2)) / (2*0.23*0.23));

    double Pgain[3] = {20,20,30};
    double Dgain[3] = {0.5,1,1};

    double posError[3];
    double velError[3];
    for (int i = 0; i < 4; i++)
    {
        if (sharedMemory->gaitTable[i] == 0)
        {
            for(int j=0; j<3; j++)
            {
                posError[j] = jointPos[j] - mMotorPosition[i*3+j];
                velError[j] = jointVel[j] - mMotorVelocity[i*3+j];
                swingtorque[i][j] = Pgain[j] * posError[j] + Dgain[j] * velError[j];
            }
        }
        else
        {
            for(int j=0; j<3; j++)
            {
                swingtorque[i][j] = 0.f;
            }
        }
    }
}

void MPCController::computeControlInput()
{
    ConvexMPCSolver.GetJacobian(robotJacobian[0],
                                mMotorPosition[0],
                                mMotorPosition[1],
                                mMotorPosition[2],1);
    ConvexMPCSolver.GetJacobian(robotJacobian[1],
                                mMotorPosition[3],
                                mMotorPosition[4],
                                mMotorPosition[5],-1);
    ConvexMPCSolver.GetJacobian(robotJacobian[2],
                                mMotorPosition[6],
                                mMotorPosition[7],
                                mMotorPosition[8],1);
    ConvexMPCSolver.GetJacobian(robotJacobian[3],
                                mMotorPosition[9],
                                mMotorPosition[10],
                                mMotorPosition[11],-1);

    for(int idx=0; idx<4; idx++)
    {
        robotJacobian[idx].transposeInPlace();
        robottorque[idx] = robotJacobian[idx]*mGRF[idx];
        mTorque[idx*3+0] = robottorque[idx][0]+swingtorque[idx][0];
        mTorque[idx*3+1] = robottorque[idx][1]+swingtorque[idx][1];
        mTorque[idx*3+2] = robottorque[idx][2]+swingtorque[idx][2];
    }
}

void MPCController::setControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        if (mTorque[index] > mTorqueLimit[index])
        {
            mTorque[index] = mTorqueLimit[index];
        }
        else if (mTorque[index] < -mTorqueLimit[index])
        {
            mTorque[index] = -mTorqueLimit[index];
        }
        sharedMemory->motorDesiredTorque[index] = mTorque[index];
    }
}
