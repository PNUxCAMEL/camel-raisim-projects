//
// Created by hs on 22. 8. 8.
//

#ifndef RAISIM_CANINEROBOT_H
#define RAISIM_CANINEROBOT_H

#include "raisim/World.hpp"
#include <Eigen/Eigen>

class CanineRobot{
public:
    CanineRobot(raisim::World *world, std::string urdfPath, std::string name){
        robotWorld = world;
        robot = world->addArticulatedSystem(urdfPath);
        robot->setName(name);
    }

    void initialize();

private:
    raisim::ArticulatedSystem *robot;
    raisim::World *robotWorld;


};



#endif //RAISIM_CANINEROBOT_H
