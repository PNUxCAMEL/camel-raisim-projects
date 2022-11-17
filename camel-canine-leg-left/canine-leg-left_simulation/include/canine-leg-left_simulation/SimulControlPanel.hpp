//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULCONTROLPANEL_HPP
#define RAISIM_SIMULCONTROLPANEL_HPP

#include <stdint.h>

#include <raisim/World.hpp>
#include <raisim/math.hpp>

#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>

#include <PDcontroller/JointPDController.hpp>
#include <IDcontroller/IDController.hpp>
#include <MPCcontroller/MPCcontroller.hpp>
#include <GRFcontroller/GRFcontroller.hpp>
#include <ControlUtils/GRFEstimatorMLP.hpp>
#include <ControlUtils/GRFEstimatorSMO.hpp>
#include <ControlUtils/GRFEstimatorETO.hpp>

class SimulControlPanel
{
public:
    SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot);

    void ControllerFunction();

private:
    void integrateSimul();
    void updateStates();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;

    uint64_t mIteration;
    uint64_t mRefMPCIteration;

    raisim::VecDyn mTorque = raisim::VecDyn(3);

    JointPDController PDcontrol;
    IDController IDcontrol;
    MPCcontroller MPCcontrol;
    GRFcontroller GRFcontrol;
    GRFEstimatorMLP GRFNet;
    GRFEstimatorSMO GRFSMO;
    GRFEstimatorETO GRFETO;

    double sumedSquaredPositionError;
    double sumedSquaredVelocityError;
    double positionRMSE;
    double velocityRMSE;
};

#endif //RAISIM_SIMULCONTROLPANEL_HPP
