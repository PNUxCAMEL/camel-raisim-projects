//
// Created by hs on 22. 10. 5.
//
#include <PDcontroller/BezierTrajectoryGenerator.hpp>

BezierTrajectoryGenerator::BezierTrajectoryGenerator()
    : mStandPosition{0.0, 0.0}
    , mSwingPosition{0.0, 0.0}
{
}

void BezierTrajectoryGenerator::InitTrajectorySet(double currentTime, double timeDuration){
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}


void BezierTrajectoryGenerator::SetCurrentTime(double currentTime) {
    mNormalizaedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int scheduler = floor(mNormalizaedTime);
    mNormalizaedTime -= scheduler;
}

void BezierTrajectoryGenerator::SwingTrajectory(double* dPos)
{
    double coeff = 0.0;
    memset(mSwingPosition, 0.0, sizeof(double)*2);
    for(int i=0; i<PNUM; i++){
        coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                * pow(mNormalizaedTime,i) * pow((1-mNormalizaedTime), (PNUM-1-i));
        mSwingPosition[0] +=  coeff * px[i];
        mSwingPosition[1] +=  coeff * pz[i];
    }
    dPos[0] = mSwingPosition[0];
    dPos[1] = mSwingPosition[1];
}

void BezierTrajectoryGenerator::StandTrajectory(double* dPos)
{
    mStandPosition[0] = -0.25*mNormalizaedTime+0.125;
    mStandPosition[1] = 1.92*pow(mStandPosition[0],2)-0.4;

    dPos[0] = mStandPosition[0];
    dPos[1] = mStandPosition[1];
}

double BezierTrajectoryGenerator::factorial(double value){
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
        result *= i;
    return result;
}