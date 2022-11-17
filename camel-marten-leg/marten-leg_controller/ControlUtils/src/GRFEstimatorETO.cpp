//
// Created by cha on 22. 11. 10.
//

#include "ControlUtils/GRFEstimatorETO.hpp"

extern pSHM sharedMemory;

GRFEstimatorETO::GRFEstimatorETO() {
    mbIsFirstRun = 1;
    mBeta = 0;
    mGain = 50.0;
    mLink1 = LINK1_LENGTH;
    mLink2 = LINK2_LENGTH;
    mMass1 = 0.221;
    mMass2 = 1.788;
    mMomentum = 0;
    mMomentumPrev = 0;
    mResidual = 0;
}

void GRFEstimatorETO::Estimate() {
    this->UpdateState();
    sharedMemory->estimatedGRFETO = -GetResidual() / LINK1_LENGTH;
}

void GRFEstimatorETO::UpdateState() {
    this->updateBeta(); ///이전에 계산해놓은 C,M 을 이용함
    this->updateCoriMat();
    this->updateMassMat();
    mMomentum = mMomentumPrev + sharedMemory->motorTorque[1] * CONTROL_dT - mBeta * CONTROL_dT + mResidual * CONTROL_dT;
    mResidual = mGain * (-mMomentum + mMassMat * sharedMemory->motorVelocity[1]);
    mMomentumPrev = mMomentum;
}

double GRFEstimatorETO::GetResidual() {
    double offset = 0;
    offset = 0.102 * 9.81;
    return mResidual - offset;
}

void GRFEstimatorETO::updateBeta() {
    double motorVelPrev;
    double motorPosPrev;
    if (mbIsFirstRun) {
        motorVelPrev = 0;
        motorPosPrev = sharedMemory->motorPosition[1];
        mbIsFirstRun = false;

    }
    mBeta = mCoriMat * motorVelPrev - (0.5 * mMass2 * mLink2 * mLink2 * sin(motorPosPrev / 2.0)) * motorVelPrev;
    motorPosPrev = sharedMemory->motorPosition[1];
    motorVelPrev = sharedMemory->motorVelocity[1];

}

void GRFEstimatorETO::updateCoriMat() {
    mCoriMat = 0.5 * mMass2 * mLink2 * mLink2 * sin(sharedMemory->motorPosition[1]);
}

void GRFEstimatorETO::updateMassMat() {
    mMassMat = 0.5 * mMass1 * mLink1 * mLink1 + mMass2 * mLink2 * mLink2 * sin(sharedMemory->motorPosition[1] / 2.0) *
                                                sin(sharedMemory->motorPosition[1] / 2.0);
}