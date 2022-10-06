//
// Created by hs on 22. 10. 5.
//
#include <PDcontroller/BezierTrajectoryGenerator.hpp>

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
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumZ = 0.0;

    double coeff = 0.0;
    for(int i=0; i<PNUM; i++)
    {
        coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                * pow(normalizedTime,i) * pow((1-normalizedTime), (PNUM-1-i));
        sumX +=  coeff * px[i];
        sumZ +=  coeff * pz[i];
    }
}

void BezierTrajectoryGenerator::setPx(double desiredVx) {
    for(int i=0; i<PNUM; i++)
    {
        px[i] = desiredVx*(0.125/2);
        if(i<2)
        {
            px[i] *= -1;
        }
    }
}
