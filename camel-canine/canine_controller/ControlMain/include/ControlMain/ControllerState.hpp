//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>
#include <PDcontroller/JointPDController.hpp>
#include <GaitScheduler/Gait.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;
    uint16_t mGaitCounter;
    JointPDController PDcontrol;

    int mHorizon;
    OffsetGait trot;
};

#endif //RAISIM_CONTROLSTATE_H
