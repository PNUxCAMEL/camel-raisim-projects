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
    RobotVisualization(raisim::World* world,
                       raisim::ArticulatedSystem* robot,
                       raisim::RaisimServer* server);
    ~RobotVisualization();
    void VisualFunction();

private:
    void openRaisimServer();
    void updateVisualReal();
    void updateVisualSimul();
    void initRobotPose();

private:
    raisim::RaisimServer* mServer;
    raisim::ArticulatedSystem* mRobot;
    raisim::World* mWorld;
    std::string mUrdfPath;

    raisim::VecDyn mTorque = raisim::VecDyn(18);
};


#endif //RAISIM_ROBOTVISUALIZATION_H
