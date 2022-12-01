//
// Created by hs on 22. 10. 24.
//
#include <ControlUtils/SwingLeg.hpp>
#include <iostream>

SwingLeg::SwingLeg(double duration)
    : mTimeDuration(duration)
{
    for (int leg=0; leg<4; leg++)
    {
        for (int pt=0; pt<PNUM; pt++)
        {
            px[leg][pt] = 0;
            py[leg][pt] = 0;
            if (pt==1 || pt==2)
            {
                pz[leg][pt] = 0.1;
            }
            else
            {
                pz[leg][pt] = 0;
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

void SwingLeg::SetControlPoints(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    px[leg][0] = initPos[0];
    py[leg][0] = initPos[1];
    pz[leg][0] = initPos[2];

    px[leg][3] = desiredPos[0];
    py[leg][3] = desiredPos[1];
    pz[leg][3] = 0;

    pz[leg][1] = pz[leg][0] + 0.1;
    pz[leg][2] = pz[leg][0] + 0.1;
    for (int idx=1; idx<3; idx++)
    {
        px[leg][idx] = px[leg][0]+idx*(px[leg][3]-px[leg][0])/3;
        py[leg][idx] = py[leg][0]+idx*(py[leg][3]-py[leg][0])/3;
    }
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
