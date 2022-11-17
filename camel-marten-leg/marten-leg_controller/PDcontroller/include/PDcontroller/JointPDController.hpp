//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>
#include <camel-tools/QuinticTrajectoryGenerator.hpp>
#include <camel-tools/SineTrajectoryGenerator.hpp>

class JointPDController
{
public:
    JointPDController();

    void DoHomeControl();
    void DoPDControl();
    void DoSineControl();
    void InitHomeStandUpTrajectory();
    void InitHomeStandDownTrajectory();
    void InitSwingTrajectory();
    void InitSineTrajectory();
    void InitCubicTrajectory();
    void InitCubicTrajectory2();

    void SetControlInput();

private:
    void setTrajectory();
    void computeControlInput();
    void updateHomeTrajectory();
    void setHomeTrajectory();
    void solveIK();
    void setSineTrajectory();


private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    QuinticTrajectoryGenerator mCubicTrjectoryGenHipVertical;
    SineTrajectoryGenerator mSineTrajectoryGenerator;
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
};


#endif //RAISIM_JOINTPDCONTROLLER_H
