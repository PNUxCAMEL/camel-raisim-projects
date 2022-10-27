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

    void DoHomeControl();
    void DoPDControl();
    void InitHomeTrajectory();
    void InitSwingTrajectory();

    void SetControlInput();

private:
    void setTrajectory();
    void computeControlInput();


private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];

    double mDesiredP[MOTOR_NUM] = {-0.125, -0.37};
    double mDesiredPosition[MOTOR_NUM];
    double mDesiredVelocity[MOTOR_NUM];
    double Kp[MOTOR_NUM];
    double Kd[MOTOR_NUM];
    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];
};



#endif //RAISIM_JOINTPDCONTROLLER_H
