//
// Created by jh on 22. 11. 3.
//

#ifndef RAISIM_IDCONTROLLER_HPP
#define RAISIM_IDCONTROLLER_HPP

#include <Eigen/Eigen>
#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>
#include <camel-tools/QuinticTrajectoryGenerator.hpp>

class IDController
{
public:
    IDController();

    void DoControl();
    void InitQuinticTrajectory();
    void SetControlInput();

private:
    void setQuinticTrajectory();
    void computeControlInput();

private:
    QuinticTrajectoryGenerator mQuinticTrajectoryGen;
    Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mTorque = Eigen::VectorXd(MOTOR_NUM);
    Eigen::VectorXd mForce = Eigen::VectorXd(2);
    double mDesiredPosition;
    double mDesiredVelocity;
    double mDesiredAcceleration;
    double Kp;
    double Kd;
    double mTorqueLimit[MOTOR_NUM];
    int mSign;

};


#endif //RAISIM_IDCONTROLLER_HPP
