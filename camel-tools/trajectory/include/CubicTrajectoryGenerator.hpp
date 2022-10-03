#ifndef RAISIM_CUBICTRAJECTORYGENERATOR_H
#define RAISIM_CUBICTRAJECTORYGENERATOR_H

#include <Eigen/Eigen>
#include <cmath>

class CubicTrajectoryGenerator {
public:
    CubicTrajectoryGenerator()
    {
        mMatrixA << 2.0, -2.0, 1.0, 1.0, -3.0, 3.0, -2.0, -1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    }
    void updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void calculateCoefficient();
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);
    double getAccelerationTrajectory(double currentTime);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 1);
    double mReferenceTime;
    double mTimeDuration;
};

class CubicTrajectoryGeneratorND {
public:
    CubicTrajectoryGeneratorND(int dim)
    {
        mMatrixA << 2.0, -2.0, 1.0, 1.0, -3.0, 3.0, -2.0, -1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        mCoefficient = Eigen::MatrixXd(dim,4);
        mFunctionValue = Eigen::MatrixXd (dim, 1);
        mDim = dim;

    }

    void updateTrajectory(Eigen::VectorXd currentPosition,Eigen::VectorXd goalPosition,double currentTime,double timeDuration);
    Eigen::VectorXd getPositionTrajectory(double currentTime);
    Eigen::VectorXd getVelocityTrajectory(double currentTime);
    Eigen::VectorXd getAccelerationTrajectory(double currentTime);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient;
    Eigen::MatrixXd mFunctionValue;
    double mReferenceTime;
    double mTimeDuration;
    int mDim;
    void calculateCoefficient();
};

class CubicTrajectoryGeneratorRotation {
public:
    CubicTrajectoryGeneratorRotation()
    {
        targetFinder = CubicTrajectoryGenerator();
    }

    void updateTrajectory(Eigen::Quaterniond currentQuaternion, Eigen::Quaterniond finalQuaternion, double currentTime, double timeDuration);
    Eigen::Vector3d getRPYPositionTrajectory(double currentTime);
    Eigen::Vector3d getRPYVelocityTrajectory(double currentTime);
    Eigen::Vector3d getRPYAccelerationTrajectory(double currentTime);
    Eigen::Quaterniond euler2quaternion(Eigen::Vector3d euler);
    Eigen::Vector3d quaternion2euler(Eigen::Quaterniond quaternion);


private:
    CubicTrajectoryGenerator targetFinder;
    Eigen::Quaterniond mStartQuaternion;
    Eigen::Quaterniond mGoalQuaternion;
    static const double delta;
};

#endif //RAISIM_CUBICTRAJECTORYGENERATOR_H
