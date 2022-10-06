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
    void PareData();
    double* GetQuaternion();
    double* GetEulerAngle();
    double* GetAccelVector();
    double* GetStabilizedAccelVector();
    double* GetMagVector();
    double* GetGyroVector();
    double* GetLinearAcceleration();
private:
    mscl::InertialNode* mNode;
    std::string mOrientQuaternion;
    ///quaternion
    double mQuaternion[4];
    /// ///mEulerAngle[0] : roll, mEulerAngle[1] : pitch, mEulerAngle[0] : yaw
    double mRoll;
    double mPitch;
    double mYaw;
    double mEulerAngle[3];
    ///모든 가속도
    double mScaledAccelX;
    double mScaledAccelY;
    double mScaledAccelZ;
    double mScaledAccelVector[3];
    ///중력가속도만 검출
    double mStabilizedAccelX;
    double mStabilizedAccelY;
    double mStabilizedAccelZ;
    double mStabilizedAccelVector[3];
    ///magnetometer
    double mScaledMagX;
    double mScaledMagY;
    double mScaledMagZ;
    double mScaledMagVector[3];
    ///gyro
    double mScaledGyroX;
    double mScaledGyroY;
    double mScaledGyroZ;
    double mScaledGyroVector[3];
    //Linear Accel
    double mEstLinearAccel[3];

};


#endif //RAISIM_LORDIMU3DMGX5AHRS_HPP
