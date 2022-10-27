//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class ControllerState{
public:
    ControllerState(raisim::World* world, raisim::ArticulatedSystem* robot);

    void ControllerFunction();

private:
    void integrateSimul();

private:
    uint16_t mIteration;
    uint16_t mGaitCounter;
    uint8_t mGaitLength;
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;

    raisim::VecDyn mTorque = raisim::VecDyn(18);

    JointPDController PDcontrol;
    MPCController MPCcontrol;
    OffsetGait stand, trot;
};

#endif //RAISIM_CONTROLSTATE_H
