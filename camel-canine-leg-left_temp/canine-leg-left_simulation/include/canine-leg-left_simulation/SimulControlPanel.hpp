//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULCONTROLPANEL_HPP
#define RAISIM_SIMULCONTROLPANEL_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <ControlUtils/GRFEstimator.hpp>

class SimulControlPanel{
public:
    SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot);

    void ControllerFunction();

private:
    void integrateSimul();
    void updateStates();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;

    uint16_t mIteration;

    raisim::VecDyn mTorque = raisim::VecDyn(3);

    JointPDController PDcontrol;
    GRFEstimator GRFNet;
};

#endif //RAISIM_SIMULCONTROLPANEL_HPP
