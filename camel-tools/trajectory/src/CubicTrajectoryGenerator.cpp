#include "../include/CubicTrajectoryGenerator.hpp"
//for debug
#include <iostream>

void CubicTrajectoryGenerator::updateTrajectory(double currentPosition, double goalPosition, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, goalPosition, 0.0, 0.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void CubicTrajectoryGenerator::calculateCoefficient() {
    mCoefficient = mMatrixA * mFunctionValue;
}

double CubicTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0) + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);
}

double CubicTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}

double CubicTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
}


void CubicTrajectoryGeneratorND::updateTrajectory(Eigen::VectorXd currentPosition, Eigen::VectorXd goalPosition, double currentTime, double timeDuration) {
    Eigen::VectorXd zeros = Eigen::VectorXd(mDim);
    zeros.setZero();

    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = goalPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void CubicTrajectoryGeneratorND::calculateCoefficient() {
    mCoefficient = mFunctionValue * mMatrixA.transpose();
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << pow(normalizedTime, 3.0), pow(normalizedTime, 2.0), normalizedTime, 1.0;
    return mCoefficient*timeMatrix;
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << 3*pow(normalizedTime, 2.0), 2*normalizedTime, 1.0, 0.0;
    return mCoefficient*timeMatrix / mTimeDuration;
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getAccelerationTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << 6*normalizedTime, 2.0, 0.0, 0.0;
    return mCoefficient*timeMatrix / pow(mTimeDuration, 2.0);
}


void CubicTrajectoryGeneratorRotation::updateTrajectory(Eigen::Quaterniond currentQuaternion, Eigen::Quaterniond finalQuaternion, double currentTime, double timeDuration) {
    mStartQuaternion = currentQuaternion;
    mGoalQuaternion = finalQuaternion;

    targetFinder.updateTrajectory(0.0, 1.0, currentTime, timeDuration);
}

Eigen::Vector3d CubicTrajectoryGeneratorRotation::getRPYPositionTrajectory(double currentTime) {
    double target = targetFinder.getPositionTrajectory(currentTime);
    Eigen::Quaterniond quaternion = mStartQuaternion.slerp(target, mGoalQuaternion);

    return quaternion2euler(quaternion);
}

Eigen::Vector3d CubicTrajectoryGeneratorRotation::getRPYVelocityTrajectory(double currentTime){
    Eigen::Vector3d currentPosition = getRPYPositionTrajectory(currentTime);
    Eigen::Vector3d previousPosition = getRPYPositionTrajectory(currentTime - delta);

    return (currentPosition - previousPosition)/delta;
}

Eigen::Vector3d CubicTrajectoryGeneratorRotation::getRPYAccelerationTrajectory(double currentTime) {
    Eigen::Vector3d currentVelocity = getRPYVelocityTrajectory(currentTime);
    Eigen::Vector3d previousVelocity = getRPYVelocityTrajectory(currentTime - delta);

    return (currentVelocity - previousVelocity)/delta;
}

Eigen::Quaterniond CubicTrajectoryGeneratorRotation::euler2quaternion(Eigen::Vector3d euler){
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return q;
}

Eigen::Vector3d CubicTrajectoryGeneratorRotation::quaternion2euler(Eigen::Quaterniond quaternion){
    return quaternion.toRotationMatrix().eulerAngles(0,1,2);
}

const double CubicTrajectoryGeneratorRotation::delta = 1e-3;
