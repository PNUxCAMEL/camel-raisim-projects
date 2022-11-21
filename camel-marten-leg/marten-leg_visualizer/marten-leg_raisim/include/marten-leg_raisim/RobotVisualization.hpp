//
// Created by jh on 22. 11. 21.
//

#ifndef RAISIM_ROBOTVISUALIZATION_HPP
#define RAISIM_ROBOTVISUALIZATION_HPP


#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>

class RobotVisualization {
public:
    RobotVisualization(raisim::World* world,
                       raisim::ArticulatedSystem* robot,
                       raisim::RaisimServer* server);
    ~RobotVisualization();
    void VisualFunction();

private:
    void openRaisimServer();
    void updateVisual();

private:
    raisim::RaisimServer* mServer;
    raisim::ArticulatedSystem* mRobot;
    raisim::World* mWorld;
};


#endif //RAISIM_ROBOTVISUALIZATION_HPP
