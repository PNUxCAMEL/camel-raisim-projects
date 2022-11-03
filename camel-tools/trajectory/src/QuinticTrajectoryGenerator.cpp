#include "../include/QuinticTrajectoryGenerator.hpp"

void QuinticTrajectoryGenerator::updateTrajectory(double currentPosition, double goalPosition, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, goalPosition, 0.0, 0.0, 0.0, 0.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void QuinticTrajectoryGenerator::calculateCoefficient() {
    mCoefficient = mMatrixA * mFunctionValue;
}

double QuinticTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCoefficient(0,0) * pow(normalizedTime, 5.0) + mCoefficient(1,0) * pow(normalizedTime, 4.0) + mCoefficient(2,0) * pow(normalizedTime, 3.0)
            + mCoefficient(3, 0) * pow(normalizedTime, 2.0) + mCoefficient(4, 0) * normalizedTime + mCoefficient(5,0);
}

double QuinticTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (5.0 *mCoefficient(0,0) * pow(normalizedTime, 4.0) + 4.0 * mCoefficient(1,0) * pow(normalizedTime, 3.0) + 3.0 * mCoefficient(2,0) * pow(normalizedTime, 2.0)
            + 2.0 * mCoefficient(3,0) * normalizedTime + mCoefficient(4,0)) / mTimeDuration;
}

double QuinticTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (20.0 *mCoefficient(0,0) * pow(normalizedTime, 3.0) + 12.0 * mCoefficient(1,0) * pow(normalizedTime, 2.0) + 6.0 * mCoefficient(2,0) * normalizedTime
            + 2.0 * mCoefficient(3,0)) / pow(mTimeDuration, 2.0);
}