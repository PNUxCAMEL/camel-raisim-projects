#include "../include/SineTrajectoryGenerator.hpp"
#include "math.h"

void SineTrajectoryGenerator::updateTrajectory(double currentPosition, double currentTime, double amplitude, double frequency){
    mReferencePose = currentPosition;
    mReferenceTime = currentTime;
    mAmplitude = amplitude;
    mFrequency = frequency;
}

double SineTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double time = (currentTime - mReferenceTime);
    return mAmplitude*sin(2*PI*mFrequency*time + PI/2)+mReferencePose-mAmplitude;
}

double SineTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double time = (currentTime - mReferenceTime);
    return 2*PI*mFrequency*mAmplitude*cos(2*PI*mFrequency*time + PI/2);
}