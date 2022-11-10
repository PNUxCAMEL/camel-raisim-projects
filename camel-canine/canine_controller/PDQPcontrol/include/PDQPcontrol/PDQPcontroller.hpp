//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_PDQPCONTROLLER_HPP
#define RAISIM_PDQPCONTROLLER_HPP

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>

//#include <WBController/QPsolver.hpp>
#include "QPsolver.hpp"

class PDQPController
{
public:
    PDQPController();

    void SetPDgain(const double& kp, const double& kd);
    void DoHomeControl();
    void InitHomeStandUpTrajectory();
    void InitHomeStandDownTrajectory();
    void SetControlInput();
    void InitTrajectory();

private:
    void updateState();
    void computeControlInput();
    void updateHomeTrajectory();
    void setHomeTrajectory();
    void setTrajectory();

private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    CubicTrajectoryGenerator mBodyTrajectory[3];
    PDQPsolver ForceQPsolver;

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
    Mat3<double> mJacobian[4];
    Vec3<double> mTorqueJacobian[4];
};



#endif //RAISIM_PDQPCONTROLLER_HPP
