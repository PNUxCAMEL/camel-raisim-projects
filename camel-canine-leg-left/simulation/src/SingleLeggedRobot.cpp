//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedRobot.hpp"

void SingleLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(GetQDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.23;  //prismatic joint
    initialJointPosition[1] = 60.0 * 3.141592 / 180.0;
    initialJointPosition[2] = -120.0 * 3.141592 / 180.0;
    SetQ(initialJointPosition);
}