//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <canineV2_util/SharedMemory.hpp>
#include <canineV2_util/RobotDescription.hpp>
#include <canineV2_util/EigenTypes.hpp>

#include <WBController/LowPDcontrol.hpp>
#include <ControlUtils/Gait.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;
    uint8_t mGaitLength;
    uint8_t mSwT;

    LowPDcontrol LowController;
    OffsetGait stand, trot, test;
};

#endif //RAISIM_CONTROLSTATE_H
