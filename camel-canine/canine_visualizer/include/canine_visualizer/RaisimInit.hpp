//
// Created by hs on 22. 8. 8.
//

#ifndef RAISIM_CANINEROBOT_H
#define RAISIM_CANINEROBOT_H

#include "raisim/World.hpp"

class RaisimInit{
public:
    RaisimInit(raisim::World* world,
               std::string urdfPath,
               std::string name,
               double dT);

    void RobotIntialize();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem *mRobot;
    raisim::Ground* mGround;
    double mDt;
};



#endif //RAISIM_CANINEROBOT_H
