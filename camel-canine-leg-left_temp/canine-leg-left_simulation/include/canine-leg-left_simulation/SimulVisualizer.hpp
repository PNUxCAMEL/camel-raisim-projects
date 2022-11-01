//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULVISUALIZER_HPP
#define RAISIM_SIMULVISUALIZER_HPP

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>

class SimulVisualizer {
public:
    SimulVisualizer(raisim::World* world,
                       raisim::ArticulatedSystem* robot,
                       raisim::RaisimServer* server);
    ~SimulVisualizer();

private:
    void initRobotPose();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;
    raisim::RaisimServer* mServer;
};

#endif //RAISIM_SIMULVISUALIZER_HPP
