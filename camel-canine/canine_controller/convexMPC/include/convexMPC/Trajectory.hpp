//
// Created by hs on 22. 7. 14.
//

#ifndef RAISIM_TRAJECTORY_H
#define RAISIM_TRAJECTORY_H

#include <cmath>

class LegTrajectory {
public:
    void updateTrajectory(double currentTime,double timeDuration);
    double getPositionTrajectory(double currentTime);

private:
    double mReferenceTime;
    double mTimeDuration;

};


#endif //RAISIM_TRAJECTORY_H
