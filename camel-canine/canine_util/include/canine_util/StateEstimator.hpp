//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "Filter.hpp"

class StateEstimator{
public:
    StateEstimator();

    void StateEstimatorFunction();
private:
    void updateState();
    void getRobotLinearState();
    void getRobotFootPosition();
private:
    CanineFilter::Vec3LPF mVelFilter;
    CanineFilter::Vec3LPF mPosFilter;

    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];
    Vec3<double> mBodyPrev[4];
    Vec3<double> mBodyPosDiff;

    bool bIsFirstRun;
    bool bIsRightFirst;
    bool bIsLeftFirst;
};

#endif //RAISIM_STATEESTIMATOR_HPP
