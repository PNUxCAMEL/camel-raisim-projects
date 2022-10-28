//
// Created by cha on 22. 10. 5.
//

#ifndef RAISIM_LORDIMU3DMGX5AHRS_HPP
#define RAISIM_LORDIMU3DMGX5AHRS_HPP

#include <iostream>
#include <string>
#include "mscl/mscl.h"


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

};


#endif //RAISIM_LORDIMU3DMGX5AHRS_HPP
