//
// Created by hs on 22. 10. 5.
//

#ifndef RAISIM_BEZIERTRAJECTORYGENERATOR_HPP
#define RAISIM_BEZIERTRAJECTORYGENERATOR_HPP

#define PNUM 12
#include <Eigen/Eigen>
#include <cmath>

class BezierTrajectoryGenerator {
public:
    BezierTrajectoryGenerator();

    void InitTrajectorySet(double currentTime,double timeDuration);
    void SetCurrentTime(double currentTime);

    void SwingTrajectory(double* dPos);
    void StandTrajectory(double* dPos);

private:
    double factorial(double value);

private:
    double mReferenceTime;
    double mTimeDuration;
    double mNormalizaedTime;

    double mSwingPosition[2];
    double mStandPosition[2];

    double pz[PNUM] = {-0.37, -0.37,
                       -0.30, -0.30, -0.30, -0.30, -0.30,
                       -0.27, -0.27, -0.27,
                       -0.37, -0.37};
    double px[PNUM] = {-0.125, -0.15,
                       -0.17, -0.17, -0.17, 0.0, 0.0,
                       0.0, 0.17, 0.17,
                       0.15, 0.125};

};


#endif //RAISIM_BEZIERTRAJECTORYGENERATOR_HPP
