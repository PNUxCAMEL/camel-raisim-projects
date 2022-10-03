//
// Created by hs on 22. 8. 8.
//

#include <canine_visualizer/CanineRobot.hpp>

void CanineRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(robot->getGeneralizedVelocityDim());
    initialJointPosition.setZero();
    initialJointVelocity.setZero();

    // base_x,y,z
    initialJointPosition[0] = 0.0;
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 0.37;

    // base_rotation [quaternion]
    initialJointPosition[3] = 1.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.0;

    // FR_hip,thigh,calf
    initialJointPosition[7] = 0.0;
    initialJointPosition[8] = 0.7;
    initialJointPosition[9] = -1.4;

    // FL_hip,thigh,calf
    initialJointPosition[10] = -0.0;
    initialJointPosition[11] = 0.7;
    initialJointPosition[12] = -1.4;

    // RR_hip,thigh,calf
    initialJointPosition[13] = 0.0;
    initialJointPosition[14] = 0.7;
    initialJointPosition[15] = -1.4;

    // RL_hip,thigh,calf
    initialJointPosition[16] = -0.0;
    initialJointPosition[17] = 0.7;
    initialJointPosition[18] = -1.4;

    robot->setGeneralizedCoordinate(initialJointPosition);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot->setGeneralizedVelocity(initialJointVelocity);
}