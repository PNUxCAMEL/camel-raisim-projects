#ifndef RAISIM_QUINTICTRAJECTORYGENERATOR_H
#define RAISIM_QUINTICTRAJECTORYGENERATOR_H

#include <Eigen/Eigen>
#include <cmath>

class QuinticTrajectoryGenerator {
public:
    QuinticTrajectoryGenerator()
    {
        mMatrixA << -6.0, 6.0, -3.0, -3.0, -0.5, 0.5,
                    15.0, -15.0, 8.0, 7.0, 1.5, -1.0,
                    -10.0, 10.0, -6.0, -4.0, -1.5, 0.5,
                    0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    void updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void calculateCoefficient();
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);
    double getAccelerationTrajectory(double currentTime);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(6, 6);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(6, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(6, 1);
    double mReferenceTime;
    double mTimeDuration;
};


#endif //RAISIM_QUINTICTRAJECTORYGENERATOR_H
