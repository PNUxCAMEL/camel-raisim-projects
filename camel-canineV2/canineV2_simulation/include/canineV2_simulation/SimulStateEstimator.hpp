//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/RobotMath.hpp>
#include <canineV2_util/Filter.hpp>

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
    CanineFilter::Vec3LPF mVelFilter;
    CanineFilter::Vec3LPF mPosFilter;

    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];
    Vec3<double> mBodyPrev[4];
    Vec3<double> mBodyPosDiff;
    bool bIsFirstRun;
    bool bIsRightFirst;
    bool bIsLeftFirst;
};

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
