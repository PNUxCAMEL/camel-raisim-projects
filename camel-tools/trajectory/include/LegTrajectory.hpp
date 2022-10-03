#ifndef RAISIM_LEGTRAJECTORY_H
#define RAISIM_LEGTRAJECTORY_H

#include <Eigen/Eigen>
#include <cmath>

class LegTrajectory {
public:
    void updateTrajectory(double currentPosition, double currentTime,double timeDuration);
    double getPositionTrajectory(double currentTime);

private:
    double mReferencePose;
    double mReferenceTime;
    double mTimeDuration;

};


#endif //RAISIM_LEGTRAJECTORY_H
