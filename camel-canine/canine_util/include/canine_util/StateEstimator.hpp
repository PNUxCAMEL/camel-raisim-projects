//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"

class StateEstimator{
public:
    StateEstimator(raisim::ArticulatedSystem* robot);

    void StateEstimatorFunction();
private:
    void EstimatorSimul();
private:
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
};

#endif //RAISIM_STATEESTIMATOR_HPP
