//
// Created by hs on 22. 10. 24.
//

#ifndef RAISIM_SWINGLEG_HPP
#define RAISIM_SWINGLEG_HPP
#define PNUM 4
#include <cmath>

class SwingLeg{
public:
    SwingLeg(double duration);
    void UpdateTrajectory(double currentTime);
    void GetPositionTrajectory(double currentTime, double* desiredPosition);

private:
    double factorial(double value);
private:
    double mReferenceTime;
    double mTimeDuration;

    double sumX = 0.0;
    double sumZ = 0.0;

    double pz[PNUM] = {-0.3, -0.15, -0.15,-0.3};
    double px[PNUM] = {0, 0, 0, 0};
};

#endif //RAISIM_SWINGLEG_HPP
