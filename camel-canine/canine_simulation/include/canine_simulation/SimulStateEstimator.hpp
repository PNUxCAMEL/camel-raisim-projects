//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

class SimulStateEstimator{
public:
    SimulStateEstimator(raisim::ArticulatedSystem* robot);

    void StateEstimatorFunction();
private:
    void updateState();
    void getJointState();
    void getRobotAngulerState();
    void getRobotLinearState();
    void getRobotFootPosition();
private:
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
    double mQuaternion[4];
    Mat4<double> mTransMat[4];
    uint8_t mFirstCount;
    uint8_t mInitCount;


    Eigen::Vector4d mBodyPosition;
    Eigen::Vector4d mBodyPositionPrev;
    Eigen::Vector4d mRightRearFootPosition;
    Eigen::Vector4d mLeftRearFootPosition;
    Eigen::Vector4d mStartBodyPosision;
    Eigen::Vector4d mWorldBodyPosition;
    Eigen::Vector4d mWorldBodyPositionPrev;
    Eigen::Vector4d mBodyVelocity;

};

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
