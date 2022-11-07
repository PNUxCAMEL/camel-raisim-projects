//
// Created by jh on 22. 11. 7.
//

#ifndef RAISIM_GRFCONTROLLER_HPP
#define RAISIM_GRFCONTROLLER_HPP

#include <Eigen/Eigen>
#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>

class GRFcontroller
{
public:
    GRFcontroller();

    void DoControl();

private:
    void computeControlInput();
    void setControlInput();


private:
    Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mTorque = Eigen::VectorXd(MOTOR_NUM);
    double mTorqueLimit[MOTOR_NUM];
    double mIntegratedError;
    double Kp;
    double Ki;
};


#endif //RAISIM_GRFCONTROLLER_HPP
