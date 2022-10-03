#ifndef RAISIM_ROBOT_HPP
#define RAISIM_ROBOT_HPP

#include"raisim/World.hpp"

class Robot
{
public:
    Robot(raisim::World* world, std::string urdfPath, std::string name);

    virtual void initialize() = 0;
    double GetWorldTime() const;
    raisim::VecDyn GetQ() const;
    raisim::VecDyn GetQD() const;
    raisim::ArticulatedSystem* GetRobot() const;

private:
    raisim::World* mRobotWorld;
    raisim::ArticulatedSystem* mRobot;

};


#endif //RAISIM_ROBOT_HPP