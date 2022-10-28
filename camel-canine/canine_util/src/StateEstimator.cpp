//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator()
{
}

void StateEstimator::StateEstimatorFunction()
{
    getJointState();
    getRobotAngulerState();
    getRobotLinearState();
    getRobotFootPosition();
}

void StateEstimator::getJointState()
{
    //TODO: Get real robot encoder value from can
}

void StateEstimator::getRobotAngulerState()
{
    //TODO: Get angular velocity, orientation from IMU
}

void StateEstimator::getRobotLinearState()
{
    //TODO: Get position, linear velocity from Kinematics and IMU, T265
}

void StateEstimator::getRobotFootPosition()
{
    //TODO: Get foot position from body frame based on kinematics
}
