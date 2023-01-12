//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/RobotDescription.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>

class JointPDController
{
public:
    JointPDController();

    void SetPDgain(const double& kp, const double& kd);
    void DoHomeControl();
    void InitHomeStandUpTrajectory();
    void InitHomeStandDownTrajectory();
    void SetControlInput();

private:
    void updateState();
    void computeControlInput();
    void updateHomeTrajectory();
    void setHomeTrajectory();


private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    int mHomeState;
    double mRefTime;
    double mDesiredPosition[MOTOR_NUM];
    double mDesiredVelocity[MOTOR_NUM];
    double Kp[MOTOR_NUM];
    double Kd[MOTOR_NUM];
    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];

    enum eHomePositionControl
    {
        HOME_NO_ACT,
        HOME_STAND_UP_PHASE1,
        HOME_STAND_UP_PHASE2,
        HOME_STAND_UP_PHASE3,
        HOME_STAND_DOWN_PHASE1,
        HOME_STAND_DOWN_PHASE2,
        HOME_STAND_DOWN_PHASE3,
    };

    Vec13<double> mInitState;
    Vec13<double> mDesiredState;
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    double mBasePosition[3];
    double mBaseVelocity[3];
    double mBaseEulerPosition[3];
    double mBaseEulerVelocity[3];
    double mFootPosition[4][3];
    Vec3<double> mGRF[4];
};



#endif //RAISIM_JOINTPDCONTROLLER_H