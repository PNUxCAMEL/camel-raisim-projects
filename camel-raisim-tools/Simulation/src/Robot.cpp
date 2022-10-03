#include "Robot.hpp"

Robot::Robot(raisim::World* world, std::string urdfPath, std::string name)
    : mRobotWorld(world)
{
    mRobot = world->addArticulatedSystem(urdfPath);
    mRobot->setName(name);
}

raisim::VecDyn Robot::GetQ() const
{
    return mRobot->getGeneralizedCoordinate();
}

raisim::VecDyn Robot::GetQD() const
{
    return mRobot->getGeneralizedVelocity();
}

double Robot::GetWorldTime() const
{
    return mRobotWorld->getWorldTime();
}

raisim::ArticulatedSystem* Robot::GetRobot() const
{
    return mRobot;
}