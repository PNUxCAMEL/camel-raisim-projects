//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"

class StateEstimator{
public:
    StateEstimator();

    void StateEstimatorFunction();
private:
    void getJointState();
    void getRobotAngulerState();
    void getRobotLinearState();
    void getRobotFootPosition();
private:

    double mQuaternion[4];
};

#endif //RAISIM_STATEESTIMATOR_HPP
