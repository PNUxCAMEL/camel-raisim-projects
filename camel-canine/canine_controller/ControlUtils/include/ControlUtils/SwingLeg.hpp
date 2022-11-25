//
// Created by hs on 22. 10. 24.
//

#ifndef RAISIM_SWINGLEG_HPP
#define RAISIM_SWINGLEG_HPP
#define PNUM 4

#include <cmath>
#include <canine_util/EigenTypes.hpp>

class SwingLeg{
public:
    SwingLeg(double duration);
    void UpdateTrajectory(double currentTime);
    void GetPositionTrajectory(double currentTime, double* desiredPosition);
    void SetControlPoints(const Vec3<double>& footPosition, const int& leg);

private:
    double factorial(double value);
private:
    double mReferenceTime;
    double mTimeDuration;

    double sumX;
    double sumY;
    double sumZ;

    double px[PNUM];
    double py[PNUM];
    double pz[PNUM];

};

#endif //RAISIM_SWINGLEG_HPP
