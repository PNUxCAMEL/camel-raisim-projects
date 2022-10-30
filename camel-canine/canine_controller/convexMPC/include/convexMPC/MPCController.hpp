//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include <canine_util/SharedMemory.hpp>

#include "MPCSolver.hpp"
#include <ControlUtils/SwingLeg.hpp>

using Eigen::Dynamic;

class MPCController{
public:
    MPCController(const uint8_t& horizon);
    void DoControl();
    void InitSwingLegTrajectory();
private:
    void updateState();
    void computeControlInput();
    void setControlInput();
    void setLegcontrol();

private:
    const uint8_t mHorizon;
    MPCSolver ConvexMPCSolver;
    SwingLeg SwingLegTrajectory;

    double mDesiredPosition[2];
    double mBasePosition[3];
    double mBaseVelocity[3];
    double mBaseEulerPosition[3];
    double mBaseEulerVelocity[3];
    double mFootPosition[4][3];
    double mMotorPosition[MOTOR_NUM];
    double mMotorVelocity[MOTOR_NUM];

    Eigen::Matrix<double,12,1> mTorque;
    double mTorqueLimit[MOTOR_NUM];

    Eigen::Matrix<double,3,1> mGRF[4];
    Eigen::Matrix<double,3,1> robottorque[4];
    Eigen::Matrix<double,3,1> swingtorque[4];
    Eigen::Matrix<double,3,3> robotJacobian[4];

};

#endif //RAISIM_MPCCONTROLLER_H
