//
// Created by hs on 22. 10. 24.
//
#include <ControlUtils/SwingLeg.hpp>

SwingLeg::SwingLeg(double duration)
    : mTimeDuration(duration)
    , px{0, 0, 0, 0}
    , py{0, 0, 0, 0}
    , pz{-0.35, -0.25, -0.25, -0.35}
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

void SwingLeg::SetControlPoints(const Vec3<double>& footPosition, const int& leg)
{
    px[0] = footPosition[0];
    py[0] = footPosition[1];
    pz[0] = footPosition[2];

    switch (leg)
    {
        case 0:
        {
            px[0] -= HIP_X_POS;
            py[0] += HIP_Y_POS;
        }
        case 1:
        {
            px[0] -= HIP_X_POS;
            py[0] -= HIP_Y_POS;
        }
        case 2: {

            px[0] += HIP_X_POS;
            py[0] += HIP_Y_POS;
        }
        case 3:
        {
            px[0] += HIP_X_POS;
            py[0] -= HIP_Y_POS;
        }
        default:
        {
            break;
        }
    }
}

void SwingLeg::GetPositionTrajectory(double currentTime, Vec3<double>& desiredPosition)
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
        sumX += coeff * px[i];
        sumY += coeff * py[i];
        sumZ += coeff * pz[i];
    }

    desiredPosition[0] = sumX;
    desiredPosition[1] = sumY;
    desiredPosition[2] = sumZ;
}
