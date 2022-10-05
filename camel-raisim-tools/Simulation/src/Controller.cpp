#include "Controller.hpp"

Controller::Controller(Robot* robot)
    : mRobot(robot)
{

}

raisim::ArticulatedSystem* Controller::GetRaisimRobot() const
{
    return mRobot->GetRobot();
}