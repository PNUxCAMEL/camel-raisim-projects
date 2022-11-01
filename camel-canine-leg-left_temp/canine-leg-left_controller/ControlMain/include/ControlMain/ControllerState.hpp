//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>

#include <PDcontroller/JointPDController.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;
    uint16_t mGaitCounter;
    uint8_t mGaitLength;

    raisim::VecDyn mTorque = raisim::VecDyn(18);

    JointPDController PDcontrol;
    MPCController MPCcontrol;
    OffsetGait stand, trot;
};

#endif //RAISIM_CONTROLSTATE_H
