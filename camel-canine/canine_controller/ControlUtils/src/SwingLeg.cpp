//
// Created by hs on 22. 10. 24.
//
#include "ControlUtils/SwingLeg.hpp"

SwingLeg::SwingLeg(double duration)
    : mTimeDuration(duration)
{
    for (int leg=0; leg<4; leg++)
    {
        for (int pt=0; pt<PNUM; pt++)
        {
            px[leg][pt] = 0;
            if (leg==0 || leg==2)
            {
                py[leg][pt] = -0.077;
            }
            else
            {
                py[leg][pt] = 0.077;
            }

            if (pt==1 || pt==2)
            {
                pz[leg][pt] = -0.2;
            }
            else
            {
                pz[leg][pt] = -0.3;
            }
        }
    }
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

void SwingLeg::GetPositionTrajectory(double currentTime, Vec3<double>& desiredPosition, const int& leg)
{
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;

    double coeff;
    for(int i=0; i<PNUM; i++)
    {
        coeff = factorial(PNUM - 1) / (factorial(i) * factorial(PNUM - 1 - i))
                * pow(normalizedTime, i) * pow((1 - normalizedTime), (PNUM - 1 - i));
        sumX += coeff * px[leg][i];
        sumY += coeff * py[leg][i];
        sumZ += coeff * pz[leg][i];
    }

    desiredPosition[0] = sumX;
    desiredPosition[1] = sumY;
    desiredPosition[2] = sumZ;
}