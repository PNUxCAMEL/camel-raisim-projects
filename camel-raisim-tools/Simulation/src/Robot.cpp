#include "Robot.hpp"

Robot::Robot(raisim::World* world, std::string urdfPath, std::string name)
    : mRobotWorld(world)
{
    mUrdfPath = URDF_RSC_DIR;
    mUrdfPath = mUrdfPath.append(urdfPath);
    mRobot = world->addArticulatedSystem(mUrdfPath);
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

std::string Robot::GetUrdfPath() const
{
    return mUrdfPath;
}

raisim::ArticulatedSystem* Robot::GetRobot() const
{
    return mRobot;
}