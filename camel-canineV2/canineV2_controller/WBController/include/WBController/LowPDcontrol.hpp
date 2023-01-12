//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_LOWPDCONTROL_HPP
#define RAISIM_LOWPDCONTROL_HPP

#include <string.h>

#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/RobotDescription.hpp>
#include <canineV2_util/EigenTypes.hpp>
#include <canineV2_util/RobotMath.hpp>

#include <ControlUtils/SwingLeg.hpp>

class LowPDcontrol{
public:
    LowPDcontrol();

    void InitSwingTrjactory();
    void DoControl();
private:
    void updateState();
    void setControlInput();
    void setLegControl();
    void getJointPos(const double& x, const double& z, Vec3<double>& pos);

private:
    const int mTorqueLimit;
    const double mSwingPgain[3];
    const double mSwingDgain[3];
    const double mStandPgain[3];
    const double mStandDgain[3];
    bool mFirstRunTrot;
    SwingLeg SwingLegTrajectory;
    Vec3<double> mLegTorque[4];
    Vec3<double> mSwingJointPos;
    Vec3<double> mSwingJointVel;
    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;

    Vec3<double> mTorque[4];

    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];

    Vec13<double> mInitState;
    Vec13<double> mDesiredState;

    Vec3<double> mDesiredPosition;

};
#endif //RAISIM_LOWPDCONTROL_HPP
