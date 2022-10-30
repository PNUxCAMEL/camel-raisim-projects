//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_WBC_HPP
#define RAISIM_WBC_HPP

#include <string.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotMath.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>

#include <WBController/QPsolver.hpp>

class WBC{
public:
    WBC();

    void InitTrajectory();
    void DoWBControl();
private:
    void updateState();
    void setTrajectory();
    void computeControlInput();
    void setControlInput();

private:
    CubicTrajectoryGenerator mCubicTrajectoryGen;
    QPsolver ForceQPsolver;

    Vec3<double> mTorque[4];
    Vec3<double> mTorqueJacobian[4];
    Vec3<double> mGRF[4];
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mTorqueLimit[4];
    Mat3<double> mJacobian[4];

    Vec13<double> mInitState;
    Vec13<double> mDesiredState;

    double mBasePosition[3];
    double mBaseVelocity[3];
    double mBaseEulerPosition[3];
    double mBaseEulerVelocity[3];
    double mFootPosition[4][3];

};
#endif //RAISIM_WBC_HPP
