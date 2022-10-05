//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTVISUALIZATION_H
#define RAISIM_ROBOTVISUALIZATION_H

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

class RobotVisualization {
public:
    RobotVisualization(raisim::World *world, raisim::RaisimServer *server);
    void visualFunction();
    void openRaisimServer();

private:
    raisim::RaisimServer *mServer;
    raisim::ArticulatedSystem *mRobot;
    raisim::World *mWorld;
    std::string mUrdfPath;

    void updateVisual();
};


#endif //RAISIM_ROBOTVISUALIZATION_H
