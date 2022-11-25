//
// Created by hs on 22. 10. 24.
//
#include <ControlUtils/SwingLeg.hpp>

SwingLeg::SwingLeg(double duration)
    : mTimeDuration(duration)
    , px{0, 0, 0, 0}
    , py{0, 0, 0, 0}
    , pz{-0.3, -0.2, -0.2, -0.3}
    , sumX(0)
    , sumY(0)
    , sumZ(0)
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

void SwingLeg::SetControlPoints(const Vec3<double>& footPosition)
{
    px[0] = footPosition[0];
    pz[0] = footPosition[2];
}

void SwingLeg::GetPositionTrajectory(double currentTime, double* desiredPosition)
{
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;

    double coeff = 0.0;
    for(int i=0; i<PNUM; i++)
    {
        coeff = factorial(PNUM - 1) / (factorial(i) * factorial(PNUM - 1 - i))
                * pow(normalizedTime, i) * pow((1 - normalizedTime), (PNUM - 1 - i));
        sumX += coeff * px[i];
        sumY += coeff * py[i];
        sumZ += coeff * pz[i];
    }

    desiredPosition[0] = sumX;
    desiredPosition[1] = sumY;
    desiredPosition[2] = sumZ;
}
