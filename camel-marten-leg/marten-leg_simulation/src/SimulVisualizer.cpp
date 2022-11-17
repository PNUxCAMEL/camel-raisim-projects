//
// Created by hs on 22. 10. 28.
//

#include <marten-leg_simulation/SimulVisualizer.hpp>

extern pSHM sharedMemory;

SimulVisualizer::SimulVisualizer(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
{
    mWorld->setGravity({ 0.0, 0.0, -9.81 });
    mWorld->setTimeStep(CONTROL_dT);
    mWorld->addGround();
    mRobot->setName("Marten-leg");
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

    initialJointPosition[0] = 0.09167;
    initialJointPosition[1] = 80 * D2R;
    initialJointPosition[2] = -160 * D2R;

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    mRobot->setGeneralizedVelocity(initialJointVelocity);
}
