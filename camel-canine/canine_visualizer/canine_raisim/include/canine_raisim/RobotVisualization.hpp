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
    RobotVisualization(raisim::World *world, std::string urdfPath, raisim::RaisimServer *server)
    {
        mWorld = world;
        mWorld->setTimeStep(0.01);
        mWorld->addGround();
        raisim::Vec<3> gravity = {0.0, 0.0, -9.81};
        mWorld->setGravity(gravity);
        mRobot = mWorld->addArticulatedSystem(urdfPath);
        mRobot->setName("Canine-leg");
        mServer = server;
    }
    void visualFunction();
    void openRaisimServer();

private:
    raisim::RaisimServer *mServer;
    raisim::ArticulatedSystem *mRobot;
    raisim::World *mWorld;
    void updateVisual();


};


#endif //RAISIM_ROBOTVISUALIZATION_H
