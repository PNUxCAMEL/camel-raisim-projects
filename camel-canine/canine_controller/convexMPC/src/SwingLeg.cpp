//
// Created by hs on 22. 10. 24.
//
#include <convexMPC/SwingLeg.hpp>
#include <iostream>

SwingLeg::SwingLeg(double duration)
    : mTimeDuration(duration)
{
}

void SwingLeg::UpdateTrajectory(double currentTime)
{
    mReferenceTime = currentTime;
}

double SwingLeg::factorial(double value)
{
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
    {
        result *= i;
    }
    return result;
}

void SwingLeg::GetPositionTrajectory(double currentTime, double* desiredPosition)
{
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumZ = 0.0;

    double coeff = 0.0;
    for(int i=0; i<PNUM; i++)
    {
        coeff = factorial(PNUM - 1) / (factorial(i) * factorial(PNUM - 1 - i))
                * pow(normalizedTime, i) * pow((1 - normalizedTime), (PNUM - 1 - i));
        sumX += coeff * px[i];
        sumZ += coeff * pz[i];
    }

    desiredPosition[0] = sumX;
    desiredPosition[1] = sumZ;
}
