//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>


class JointPDController
{
public:
    JointPDController();

    void SetPDgain(const double& kp, const double& kd);
    void DoHomeControl();
    void DoPDControl();
    void InitHomeStandUpTrajectory();
    void InitHomeStandDownTrajectory();
    void InitSwingTrajectory();

    void SetControlInput();

private:
    void setTrajectory();
    void computeControlInput();
    void updateHomeTrajectory();
    void setHomeTrajectory();


private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    int mHomeState;
    double mRefTime;
    double mDesiredP[MOTOR_NUM] = {-0.125, -0.37};
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
