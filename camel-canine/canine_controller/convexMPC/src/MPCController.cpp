//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , ConvexMPCSolver(mHorizon)
{
    for(double & motorIdx : mTorqueLimit)
    {
        motorIdx = 50.0;
    }

    mGRF->setZero();
    mTorque.setZero();
    robotJacobian->setZero();
    robottorque->setZero();
}

void MPCController::DoControl()
{
    //TODO: Make structure for robot states
    updateState();
    ConvexMPCSolver.SetTrajectory(mBasePosition, mBaseEulerPosition);
    ConvexMPCSolver.GetMetrices(mBasePosition, mBaseEulerPosition,
                                mBaseVelocity, mBaseEulerVelocity,
                                mFootPosition);
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    computeControlInput();
    setControlInput();
}

void MPCController::updateState()
{
    memcpy(mBasePosition, sharedMemory->basePosition, sizeof(double)*3);
    memcpy(mBaseVelocity, sharedMemory->baseVelocity, sizeof(double)*3);
    memcpy(mBaseEulerPosition, sharedMemory->baseEulerPosition, sizeof(double)*3);
    memcpy(mBaseEulerVelocity, sharedMemory->baseEulerVelocity, sizeof(double)*3);
    memcpy(mFootPosition, sharedMemory->footPosition, sizeof(double)*4*3);
    memcpy(mMotorPosition, sharedMemory->motorPosition, sizeof(double)*MOTOR_NUM);
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
        mTorque[idx*3  ] = robottorque[idx][0];
        mTorque[idx*3+1] = robottorque[idx][1];
        mTorque[idx*3+2] = robottorque[idx][2];
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
        std::cout << mTorque[index] << "\t";
    }
    std::cout << std::endl;
}
