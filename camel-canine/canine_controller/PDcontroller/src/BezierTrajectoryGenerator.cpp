//
// Created by hs on 22. 10. 5.
//
#include <PDcontroller/BezierTrajectoryGenerator.hpp>
#include <iostream>

void BezierTrajectoryGenerator::updateTrajectory(double currentTime, double timeDuration){
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

double BezierTrajectoryGenerator::factorial(double value){
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
        result *= i;
    return result;
}

void BezierTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    mNormalizaedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int scheduler = floor(mNormalizaedTime);
    mNormalizaedTime -= scheduler;
}

void BezierTrajectoryGenerator::SwingTrajectory()
{
    double coeff = 0.0;
    swingX = 0.0;
    swingZ = 0.0;
    for(int i=0; i<PNUM; i++){
        coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                * pow(mNormalizaedTime,i) * pow((1-mNormalizaedTime), (PNUM-1-i));
        swingX +=  coeff * px[i];
        swingZ +=  coeff * pz[i];
    }
}

void BezierTrajectoryGenerator::StandTrajectory()
{
    standX = -0.25*mNormalizaedTime+0.125;
    standZ = 1.92*pow(standX,2)-0.4;
}