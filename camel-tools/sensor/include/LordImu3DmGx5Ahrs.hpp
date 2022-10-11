//
// Created by cha on 22. 10. 5.
//

#ifndef RAISIM_LORDIMU3DMGX5AHRS_HPP
#define RAISIM_LORDIMU3DMGX5AHRS_HPP

#include <iostream>
#include <string>
#include "mscl/mscl.h"
#include "camel-tools/ThreadGenerator.hpp"


class LordImu3DmGx5Ahrs
{
public:
    LordImu3DmGx5Ahrs(mscl::InertialNode* node);

    void SetConfig(int samplingHz);
    void ParseData();
    double* GetQuaternion();
    double* GetEulerAngle();
    double* GetAccelVector();
    double* GetStabilizedAccelVector();
    double* GetMagVector();
    double* GetAngularVelocity();
    double* GetLinearAcceleration();
    double* GetLinearVelocity();

private:
    mscl::InertialNode* mNode;
    std::string mOrientQuaternion;
    ///quaternion
    double mQuaternion[4];
    /// ///mEulerAngle[0] : roll, mEulerAngle[1] : pitch, mEulerAngle[0] : yaw
    double mEulerAngle[3];
    ///all acceleration
    double mScaledAccelVector[3];
    ///a
    double mStabilizedAccelVector[3];
    ///magnetometer
    double mScaledMagVector[3];
    ///angular Velocity
    double mAngularVelocity[3];
    ///Linear Accel
    double mEstLinearAccel[3];
    ///linera vel
    double mLinearVelocity[3];
    double mPrevLinearVelocity[3];

    struct timespec TIME_NOW;
    struct timespec TIME_PREV;
};


#endif //RAISIM_LORDIMU3DMGX5AHRS_HPP
