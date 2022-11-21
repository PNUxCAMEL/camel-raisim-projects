//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <IDcontroller/IDController.hpp>
#include <MPCcontroller/MPCcontroller.hpp>

class ControllerState
{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint64_t mIteration;

    raisim::VecDyn mTorque = raisim::VecDyn(2);

    JointPDController PDcontrol;
    IDController IDcontrol;
    MPCcontroller MPCcontrol;
};

#endif //RAISIM_CONTROLSTATE_H
