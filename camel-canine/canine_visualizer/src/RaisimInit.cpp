//
// Created by hs on 22. 8. 8.
//

#include <canine_visualizer/RaisimInit.hpp>

RaisimInit::RaisimInit(raisim::World *world, std::string urdfPath, std::string name, double dT)
    : mWorld(world)
    , mRobot(mWorld->addArticulatedSystem(urdfPath))
    , mGround(mWorld->addGround(0,"gnd"))
    , mDt(dT)
{
    mWorld->setTimeStep(mDt);
    mRobot->setName(name);
}

void RaisimInit::RobotIntialize() {
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(mRobot->getGeneralizedVelocityDim());
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

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    mRobot->setGeneralizedVelocity(initialJointVelocity);
}