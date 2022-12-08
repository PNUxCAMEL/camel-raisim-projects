//
// Created by hs on 22. 10. 24.
//

#ifndef RAISIM_SWINGLEG_HPP
#define RAISIM_SWINGLEG_HPP
#define PNUM 4
#include <cmath>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotDescription.hpp>

#define PNUM 4

class SwingLeg{
public:
    SwingLeg(double duration);
    void UpdateTrajectory(double currentTime);
    void GetPositionTrajectory(double currentTime, Vec3<double>& desiredPosition, const int& leg);

private:
    double factorial(double value);
private:
    double mReferenceTime;
    double mTimeDuration;

    double sumX;
    double sumY;
    double sumZ;

    double px[4][PNUM];
    double py[4][PNUM];
    double pz[4][PNUM];
};

#endif //RAISIM_SWINGLEG_HPP
