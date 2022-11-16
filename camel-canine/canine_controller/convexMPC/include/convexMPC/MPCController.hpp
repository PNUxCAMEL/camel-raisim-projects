//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include <ControlUtils/SwingLeg.hpp>

#include "MPCSolver.hpp"


using Eigen::Dynamic;

class MPCController{
public:
    MPCController(const uint8_t& horizon, const double& swingT);
    void DoControl();
    void InitUpTrajectory();
    void InitDownTrajectory();
    void InitSwingTrjactory();
private:
    void updateState();
    void computeControlInput();
    void setControlInput();
    void setLegControl();
    void getJointPos(const double& x, const double& z, Vec3<double>& pos);

private:
    const uint8_t mHorizon;
    const int mTorqueLimit;
    const double mSwingPgain[3];
    const double mSwingDgain[3];
    const double mStandPgain[3];
    const double mStandDgain[3];
    MPCSolver ConvexMPCSolver;
    SwingLeg SwingLegTrajectory;
    CubicTrajectoryGenerator mBaseTrajectory[3];

    double mDesiredPosition[2];
    double mBasePosition[3];
    double mBaseVelocity[3];
    double mBaseEulerPosition[3];
    double mBaseEulerVelocity[3];
    double mFootPosition[4][3];


    Vec3<double> mGRF[4];
    Vec3<double> mTorque[4];
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mTorqueJacobian[4];
    Vec13<double> mInitState;
    Mat3<double> mJacobian[4];

    Vec3<double> mLegTorque[4];
    Vec3<double> mSwingJointPos;
    Vec3<double> mSwingJointVel;
    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;

    bool mFirstRunTrot;
};

#endif //RAISIM_MPCCONTROLLER_H
