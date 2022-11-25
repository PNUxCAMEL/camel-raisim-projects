//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_LOWPDCONTROL_HPP
#define RAISIM_LOWPDCONTROL_HPP

#include <string.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotMath.hpp>

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
    void getJointPos(Vec3<double>& resultPos, Vec3<double> desiredPos, const int& leg, const bool stand);

private:
    const int mTorqueLimit;

    bool mFirstRunTrot;
    SwingLeg SwingLegTrajectory;
    Vec3<double> mLegTorque[4];
    Vec3<double> mSwingJointPos;
    Vec3<double> mSwingJointVel;
    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;
    Vec3<double> mSwingPgain;
    Vec3<double> mSwingDgain;
    Vec3<double> mStandPgain;
    Vec3<double> mStandDgain;

    Vec3<double> mTorque[4];

    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mFootPosition[4];

    Vec13<double> mInitState;
    Vec13<double> mDesiredState;

    Vec3<double> mDesiredPosition;

};
#endif //RAISIM_LOWPDCONTROL_HPP
