//
// Created by jaehoon on 22. 6. 1.
//
#include <random>
#include <iostream>
#include <Eigen/Eigen>
#include "camel-tools/trajectory.hpp"

static const double rad2deg = 180.0 / 3.141592;
static const double deg2rad = 3.141592 / 180.0;


void CubicTrajectoryTest(double currentPosition, double goalPosition, double currentTime, double timeDuration)
{
    CubicTrajectoryGenerator trajGen;
    trajGen.updateTrajectory(currentPosition, goalPosition, currentTime, timeDuration);
    double realTime;
    double accumulatedPosition = 0.0;
    double accumulatedVelocity = 0.0;
    double dT = 0.001;
    for (int i = 0; i < timeDuration / dT + 1; i++)
    {
        realTime = i * dT;
        std::cout << "iteration : " << i << std::endl;
        std::cout << "desired position : " << trajGen.getPositionTrajectory(realTime) << std::endl;
        std::cout << "desired velocity : " << trajGen.getVelocityTrajectory(realTime) << std::endl;
        std::cout << "desired acceleration : " << trajGen.getAccelerationTrajectory(realTime) << std::endl;
        accumulatedPosition += trajGen.getVelocityTrajectory(realTime) * dT;
        accumulatedVelocity += trajGen.getAccelerationTrajectory(realTime) * dT;
        std::cout << "accumulated position : " << accumulatedPosition << std::endl;
        std::cout << "accumulated velocity : " << accumulatedVelocity << std::endl;
    }
}

void QuinticTrajectoryTest(double currentPosition, double goalPosition, double currentTime, double timeDuration)
{
    QuinticTrajectoryGenerator trajGen;
    trajGen.updateTrajectory(currentPosition, goalPosition, currentTime, timeDuration);
    double realTime;
    double accumulatedPosition = 0.0;
    double accumulatedVelocity = 0.0;
    double dT = 0.001;
    for (int i = 0; i < timeDuration / dT + 1; i++)
    {
        realTime = i * dT;
        std::cout << "iteration : " << i << std::endl;
        std::cout << "desired position : " << trajGen.getPositionTrajectory(realTime) << std::endl;
        std::cout << "desired velocity : " << trajGen.getVelocityTrajectory(realTime) << std::endl;
        std::cout << "desired acceleration : " << trajGen.getAccelerationTrajectory(realTime) << std::endl;
        accumulatedPosition += trajGen.getVelocityTrajectory(realTime) * dT;
        accumulatedVelocity += trajGen.getAccelerationTrajectory(realTime) * dT;
        std::cout << "accumulated position : " << accumulatedPosition << std::endl;
        std::cout << "accumulated velocity : " << accumulatedVelocity << std::endl;
    }
}
//
// Created by jaehoon on 22. 8. 19.
//

void printRotationTest(CubicTrajectoryGeneratorRotation* rotationTest, double realTime, Eigen::Vector3d* accumulatedPosition, Eigen::Vector3d* accumulatedVelocity, double dT)
{
    std::cout << "desired position : " << (*rotationTest).getRPYPositionTrajectory(realTime).transpose() << std::endl;
    std::cout << "desired velocity : " << (*rotationTest).getRPYVelocityTrajectory(realTime).transpose() << std::endl;
    std::cout << "desired acceleration : " << (*rotationTest).getRPYAccelerationTrajectory(realTime).transpose() << std::endl;

    *accumulatedPosition += (*rotationTest).getRPYVelocityTrajectory(realTime) * dT;
    *accumulatedVelocity += (*rotationTest).getRPYAccelerationTrajectory(realTime) * dT;

    std::cout << "accumulatePosition : " << (*accumulatedPosition).transpose() << std::endl;
    std::cout << "accumulateVelocity : " << (*accumulatedVelocity).transpose() << std::endl << std::endl;
}

void CubicTrajectoryGeneratorRotationTest(Eigen::Quaterniond currentQuaternion, Eigen::Quaterniond finalQuaternion, double currentTime, double timeDuration)
{
    double dT = 0.001;

    CubicTrajectoryGeneratorRotation rotationTest = CubicTrajectoryGeneratorRotation();
    rotationTest.updateTrajectory(currentQuaternion, finalQuaternion, currentTime, timeDuration);
    double realTime = 0.0;
    Eigen::Vector3d accumulatedPosition = Eigen::Vector3d().setZero();
    Eigen::Vector3d accumulatedVelocity = Eigen::Vector3d().setZero();
    while (realTime < currentTime + timeDuration)
    {
        printRotationTest(&rotationTest, realTime, &accumulatedPosition, &accumulatedVelocity, dT);
        realTime += dT;
    }
    double a = 1e-7;
    std::cout << a << std::endl;
//    printRotationTest(&rotationTest, realTime, &accumulatedPosition, &accumulatedVelocity, dT);
}


Eigen::Quaterniond euler2quaternion(Eigen::Vector3d euler)
{
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return q;
}


int main()
{
//    CubicTrajectoryTest(0.0, 0.53, 0.0, 5.0);
//    QuinticTrajectoryTest(0.0, 0.53, 0.0, 5.0);
//    double position = 0.23;
//    std::cout<<acos(position / 0.46)<<std::endl;

    Eigen::Quaterniond currentQuaternion = euler2quaternion({ 0.0 * deg2rad, 0.0 * deg2rad, 0.0 * deg2rad });
    Eigen::Quaterniond finalQuaternion = euler2quaternion({ 90.0 * deg2rad, 90.0 * deg2rad, 90.0 * deg2rad });
    double currentTime = 0.0;
    double timeDuration = 5.0;
    CubicTrajectoryGeneratorRotationTest(currentQuaternion, finalQuaternion, currentTime, timeDuration);


//    for (int i = 0; i < 100; i++)
//    {
//        randomGoalPosition = double(dis(gen)) / 100.0 * 0.15 + 0.23;
//        std::cout << "Ramdom Num : " << randomGoalPosition << std::endl;
//    }
}