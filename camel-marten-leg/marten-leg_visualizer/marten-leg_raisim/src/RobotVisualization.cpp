//
// Created by jh on 22. 11. 21.
//

#include <marten-leg_raisim/RobotVisualization.hpp>

extern pSHM sharedMemory;

RobotVisualization::RobotVisualization(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(VISUAL_dT);
    mWorld->addGround();
    mRobot->setName("Marten-leg");
}

RobotVisualization::~RobotVisualization()
{
    mServer->killServer();
}

void RobotVisualization::VisualFunction()
{
    switch(sharedMemory->visualState)
    {
    case STATE_VISUAL_STOP:
    {
        break;
    }
    case STATE_OPEN_RAISIM:
    {
        openRaisimServer();
        sharedMemory->visualState = STATE_UPDATE_VISUAL;
        break;
    }
    case STATE_UPDATE_VISUAL:
    {
        updateVisual();
        break;
    }
    default:
        break;
    }
}

void RobotVisualization::openRaisimServer()
{
    mServer->launchServer(8080);
    sleep(1);
}

void RobotVisualization::updateVisual()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();

    // base_x,y,z
    initialJointPosition[0] = sharedMemory->hipVerticalPosition+0.015;
    initialJointPosition[1] = sharedMemory->motorPosition[HIP_IDX];
    initialJointPosition[2] = sharedMemory->motorPosition[KNEE_IDX];
    mRobot->setGeneralizedCoordinate(initialJointPosition);
}