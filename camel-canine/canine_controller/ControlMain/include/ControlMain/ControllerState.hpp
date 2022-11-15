//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <WBController/WBC.hpp>
#include <PDcontroller/JointPDController.hpp>
#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;
    uint8_t mGaitLength;

    JointPDController PDcontrol;
    MPCController MPCcontrol;
    WBC WBControl;

    OffsetGait stand, trot, test;
};

#endif //RAISIM_CONTROLSTATE_H
