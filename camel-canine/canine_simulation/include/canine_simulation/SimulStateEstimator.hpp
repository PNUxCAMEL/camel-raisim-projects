//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

class SimulStateEstimator{
public:
    SimulStateEstimator(raisim::ArticulatedSystem* robot);

    void StateEstimatorFunction();
private:
    void getJointState();
    void getRobotAngulerState();
    void getRobotLinearState();
    void getRobotFootPosition();
private:
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
    raisim::Vec<3> mFootPosition[4];
    double mQuaternion[4];
};

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
