//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot)
        : mRobot(robot)
        , mPosition(raisim::VecDyn(19))
        , mVelocity(raisim::VecDyn(18))
        , mFirstCount(0)

{
}

void SimulStateEstimator::StateEstimatorFunction()
{
    updateState();
    getJointState();
    getRobotAngulerState();
    getRobotLinearState();
    getRobotFootPosition();
}

void SimulStateEstimator::updateState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    TransformationBody2Foot(&mTransMat[0], R_FRON, sharedMemory->motorPosition[0],
                                                   sharedMemory->motorPosition[1],
                                                   sharedMemory->motorPosition[2]);
    TransformationBody2Foot(&mTransMat[1], L_FRON, sharedMemory->motorPosition[3],
                                                   sharedMemory->motorPosition[4],
                                                   sharedMemory->motorPosition[5]);
    TransformationBody2Foot(&mTransMat[2], R_BACK, sharedMemory->motorPosition[6],
                                                   sharedMemory->motorPosition[7],
                                                   sharedMemory->motorPosition[8]);
    TransformationBody2Foot(&mTransMat[3], L_BACK, sharedMemory->motorPosition[9],
                                                   sharedMemory->motorPosition[10],
                                                   sharedMemory->motorPosition[11]);
}

void SimulStateEstimator::getJointState()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void SimulStateEstimator::getRobotAngulerState()
{
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->baseEulerVelocity[idx] = mVelocity[idx+3];
    }

    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = mPosition[idx+3];
    }
    TransformQuat2Euler(mQuaternion, sharedMemory->baseEulerPosition);
}

void SimulStateEstimator::getRobotLinearState()
{
    if(mInitCount<5)
    {
        for (int idx=0; idx<3; idx++)
        {
            sharedMemory->basePosition[idx] = mTransMat[2].inverse()(idx,3);
            sharedMemory->baseVelocity[idx] = 0;
            sharedMemory->initPosition[idx] = sharedMemory->basePosition[idx];

            std::cout << sharedMemory->initPosition[idx] << "\t";
        }
        std::cout << std::endl;
        mInitCount++;
    }
    else
    {
        double tempDiff;
        for (int idx=0; idx<3; idx++)
        {
            tempDiff = mTransMat[2].inverse()(idx,3)-sharedMemory->basePosition[idx];
            sharedMemory->basePosition[idx] += tempDiff;
            sharedMemory->baseVelocity[idx] = tempDiff/ESTIMATOR_dT;
        }
    }
}

void SimulStateEstimator::getRobotFootPosition()
{
    for (int leg=0; leg<4; leg++)
    {
        for (int idx=0; idx<3; idx++)
        {
            sharedMemory->footPosition[leg][idx] = mTransMat[leg](idx,3);
        }
    }
}
