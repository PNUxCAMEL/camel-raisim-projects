//
// Created by hs on 22. 10. 28.
//

#include <canine_simulation/SimulVisualizer.hpp>

extern pSHM sharedMemory;

SimulVisualizer::SimulVisualizer(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
        : mWorld(world)
        , mRobot(robot)
        , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(CONTROL_dT);
    mWorld->addGround();
    mRobot->setName("Canine");
    initRobotPose();
}

SimulVisualizer::~SimulVisualizer()
{
    mServer->killServer();
}

void SimulVisualizer::initRobotPose()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(mRobot->getGeneralizedVelocityDim());
    initialJointPosition.setZero();
    initialJointVelocity.setZero();

    initialJointPosition[2] = 0.0578;
    initialJointPosition[3] = 1.0;

    for (int idx=0; idx<4; idx++)
    {
        initialJointPosition[idx*3+7] = 0.0;
        initialJointPosition[idx*3+8] = 2.5;
        initialJointPosition[idx*3+9] = -2.9;
    }

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    mRobot->setGeneralizedVelocity(initialJointVelocity);
}
