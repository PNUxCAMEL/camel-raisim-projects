//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include <canine_util/SharedMemory.hpp>

#include "MPCSolver.hpp"

using Eigen::Dynamic;

class MPCController{
public:
    MPCController(const uint8_t& horizon);
    void DoControl();
private:
    void computeControlInput();
    void setControlInput();

private:
    const uint8_t mHorizon;
    MPCSolver ConvexMPCSolver;

    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];

    Eigen::Matrix<double,3,1> mGRF[4];
    Eigen::Matrix<double,3,1> robottorque[4];
    Eigen::Matrix<double,3,3> robotJacobian[4];

};

#endif //RAISIM_MPCCONTROLLER_H
