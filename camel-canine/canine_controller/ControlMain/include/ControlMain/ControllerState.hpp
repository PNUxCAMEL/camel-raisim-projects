//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_CONTROLLERSTATE_HPP
#define RAISIM_CONTROLLERSTATE_HPP

#include <stdint.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <PDcontroller/JointPDController.hpp>

class ControllerState{
public:
    ControllerState();

    void ControllerFunction();

private:
    uint16_t mIteration;

    JointPDController PDcontrol;
};

#endif //RAISIM_CONTROLSTATE_H
