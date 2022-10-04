#include "Robot.hpp"

Robot::Robot(raisim::World* world, std::string urdfPath, std::string name)
    : mRobotWorld(world)
{
    mRobot = world->addArticulatedSystem(urdfPath);
    mRobot->setName(name);
}

void Robot::SetQ(Eigen::VectorXd Q) const
{
    mRobot->setGeneralizedCoordinate(Q);
}

void Robot::SetTau(Eigen::VectorXd tau) const
{
    mRobot->setGeneralizedForce(tau);
}

int Robot::GetQDim() const
{
    return mRobot->getGeneralizedCoordinateDim();
}

int Robot::GetQDDim() const
{
    return mRobot->getGeneralizedVelocityDim();
}

double Robot::GetWorldTime() const
{
    return mRobotWorld->getWorldTime();
}

Eigen::VectorXd Robot::GetQ() const
{
    return mRobot->getGeneralizedCoordinate().e();
}

Eigen::VectorXd Robot::GetQD() const
{
    return mRobot->getGeneralizedVelocity().e();
}

Eigen::VectorXd Robot::GetCOM() const
{
    return mRobot->getCOM().e();
}

Eigen::MatrixXd Robot::GetMassMatrix() const
{
    return mRobot->getMassMatrix().e();
}

raisim::ArticulatedSystem* Robot::GetRobot() const
{
    return mRobot;
}