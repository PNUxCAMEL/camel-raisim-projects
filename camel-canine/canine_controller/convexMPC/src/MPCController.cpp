//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , ConvexMPCSolver(mHorizon)
    , SwingLegTrajectory(0.25)
{
    for(double & motorIdx : mTorqueLimit)
    {
        motorIdx = 50.0;
    }
    mGRF->setZero();
    mTorque.setZero();
    robotJacobian->setZero();
    robottorque->setZero();
    swingtorque->setZero();
}

void MPCController::InitSwingLegTrajectory()
{
    SwingLegTrajectory.UpdateTrajectory(sharedMemory->localTime);
}

void MPCController::DoControl()
{
    //TODO: Make structure for robot states
    updateState();
    ConvexMPCSolver.SetTrajectory(mBasePosition);
    ConvexMPCSolver.GetMetrices(mBasePosition, mBaseEulerPosition,
                                mBaseVelocity, mBaseEulerVelocity,
                                mFootPosition);
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    setLegcontrol();
    computeControlInput();
    setControlInput();
}

void MPCController::updateState()
{
//    memcpy(mBasePosition, sharedMemory->basePosition, sizeof(double)*3);
//    memcpy(mBaseVelocity, sharedMemory->baseVelocity, sizeof(double)*3);
//    memcpy(mBaseEulerPosition, sharedMemory->baseEulerPosition, sizeof(double)*3);
//    memcpy(mBaseEulerVelocity, sharedMemory->baseEulerVelocity, sizeof(double)*3);
//    memcpy(mFootPosition, sharedMemory->footPosition, sizeof(double)*4*3);
//    memcpy(mMotorPosition, sharedMemory->motorPosition, sizeof(double)*MOTOR_NUM);
//    memcpy(mMotorVelocity, sharedMemory->motorVelocity, sizeof(double)*MOTOR_NUM);

    for(int i = 0; i < 3 ; i++)
    {
        mBasePosition[i] = sharedMemory->basePosition[i];
        mBaseVelocity[i] =  sharedMemory->baseVelocity[i];
        mBaseEulerPosition[i] =  sharedMemory->baseEulerPosition[i];
        mBaseEulerVelocity[i] =  sharedMemory->baseEulerVelocity[i];
    }

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            mFootPosition[i][j] = sharedMemory->footPosition[i][j];
        }
    }

    for(int i = 0; i < MOTOR_NUM ; i++)
    {
        mMotorPosition[i] = sharedMemory->motorPosition[i];
        mMotorVelocity[i] = sharedMemory->motorVelocity[i];
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

    double Pgain[3] = {5,20,30};
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
