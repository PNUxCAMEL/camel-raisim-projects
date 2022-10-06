//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

#include "BezierTrajectoryGenerator.hpp"
#include "CubicTrajectoryGenerator.hpp"

class JointPDController
{
public:
    JointPDController();


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

public:
    void controllerFunction();

private:
    void setPDGain(double Kp[MOTOR_NUM], double Kd[MOTOR_NUM]);
    void setTrajectory();
    void computeControlInput();
    void setControlInput();
    void doHomeControl();
    void doPDControl();
};



#endif //RAISIM_JOINTPDCONTROLLER_H
