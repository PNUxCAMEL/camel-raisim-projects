//
// Created by jh on 22. 11. 3.
//

#include <ControlUtils/GRFEstimatorSMO.hpp>
#include <iostream>

extern pSHM sharedMemory;

GRFEstimatorSMO::GRFEstimatorSMO()
{
    mJacobian.setZero();
    mTorque.setZero();
}

void GRFEstimatorSMO::Estimate()
{
    double dx_dth1;
    double dx_dth2;
    double dz_dth1;
    double dz_dth2;
    dx_dth1 = LINK2_LENGTH * cos(sharedMemory->motorPosition[0]) + LINK1_LENGTH * cos(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dx_dth2 = LINK1_LENGTH * cos(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dz_dth1 = -LINK2_LENGTH * sin(sharedMemory->motorPosition[0]) - LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dz_dth2 = -LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    mJacobian << dx_dth1, dx_dth2, dz_dth1, dz_dth2;
    mTorque[0] = sharedMemory->motorDesiredTorque[0];
    mTorque[1] = sharedMemory->motorDesiredTorque[1];
    sharedMemory->estimatedGRFSMO = ((mJacobian * mJacobian.transpose()).inverse() * mJacobian * mTorque)[1];
}

