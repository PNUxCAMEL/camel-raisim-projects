//
// Created by hs on 22. 7. 14.
//

#include "SwingLegTrajectory.h"
#include <iostream>

void SwingLegTrajectory::updateTrajectory(double currentTime, double timeDuration){
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

double SwingLegTrajectory::factorial(double value){
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
    {
        result *= i;
    }
    return result;
}

void SwingLegTrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;

    double coeff = 0.0;
    for(int i=0; i<PNUM; i++)
    {
        coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                * pow(normalizedTime,i) * pow((1-normalizedTime), (PNUM-1-i));
        sumX +=  coeff * px[i];
        sumY +=  coeff * py[i];
        sumZ +=  coeff * pz[i];
    }

}

void SwingLegTrajectory::setPx(double desiredVx, double desiredVy) {
    for(int i=0; i<PNUM; i++)
    {
        px[i] = desiredVx*(0.125/2);
        py[i] = desiredVy*(0.125/2);
        if(i<2)
        {
            px[i] *= -1;
            py[i] *= -1;
        }
    }
}

