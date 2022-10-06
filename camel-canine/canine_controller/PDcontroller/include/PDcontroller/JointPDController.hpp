//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

#include <camel-tools/CubicTrajectoryGenerator.hpp>

#include "BezierTrajectoryGenerator.hpp"

class JointPDController
{
public:
    JointPDController();

    void DoHomeControl();
    void DoPDControl();
    void SetPDGain(double Kp[MOTOR_NUM], double Kd[MOTOR_NUM]);
    void InitHomeTrajectory();
    void InitSwingTrajectory();
    void setTrajectory();
    void computeControlInput();
    void setControlInput();

private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    BezierTrajectoryGenerator mBezierTrajectoryGen;

    int mIteration;
    double mDesiredP[MOTOR_NUM] = {-0.125, -0.37};
    double mDesiredPosition[MOTOR_NUM];
    double mDesiredVelocity[MOTOR_NUM];
    double Kp[MOTOR_NUM];
    double Kd[MOTOR_NUM];
    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];
};



#endif //RAISIM_JOINTPDCONTROLLER_H
