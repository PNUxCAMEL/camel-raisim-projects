//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>
#include <canine_util/Filter.hpp>

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
    CanineFilter::Vec3LPF mPosFilter;
    CanineFilter::Vec3LPF mVelFilter;

    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];
    Vec3<double> mTempPosPrev;
    Vec3<double> mTempPos[4];
    Vec3<double> mTempPosMean;
    Vec3<double> mTempVel;

    Vec3<double> mInitPosition[4];
    bool bIsFirstRun;
    int mStandCount;

};

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
