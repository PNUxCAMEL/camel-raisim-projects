#include "Controller.hpp"

Controller::Controller(Robot* robot)
    : mRobot(robot)
{

}

Robot* Controller::GetRobot() const
{
    return mRobot;
}