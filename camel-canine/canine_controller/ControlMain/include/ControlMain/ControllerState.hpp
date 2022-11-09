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
#include <PDQPcontrol/PDQPcontroller.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;
    uint16_t mGaitCounter;
    uint8_t mGaitLength;

    JointPDController PDcontrol;
    MPCController MPCcontrol;
    PDQPController PDQPcontrol;
    WBC WBControl;

    OffsetGait stand, trot;
};

#endif //RAISIM_CONTROLSTATE_H
