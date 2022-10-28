//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULCONTROLPANEL_HPP
#define RAISIM_SIMULCONTROLPANEL_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class SimulControlPanel{
public:
    SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot);

    void ControllerFunction();

private:
    void integrateSimul();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;

    uint16_t mIteration;
    uint16_t mGaitCounter;
    uint8_t mGaitLength;

    raisim::VecDyn mTorque = raisim::VecDyn(18);

    JointPDController PDcontrol;
    MPCController MPCcontrol;
    OffsetGait stand, trot;
};

#endif //RAISIM_SIMULCONTROLPANEL_HPP