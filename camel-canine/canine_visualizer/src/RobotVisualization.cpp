//
// Created by camel on 22. 9. 21.
//

#include <canine_visualizer/RobotVisualization.hpp>

extern pSHM sharedMemory;

void RobotVisualization::openRaisimServer()
{
    mServer->launchServer(8080);
    sleep(1);
}

void RobotVisualization::updateVisual()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.75;
    initialJointPosition[1] = sharedMemory->motorPosition[HIP_IDX];
    initialJointPosition[2] = sharedMemory->motorPosition[KNEE_IDX];
    mRobot->setGeneralizedCoordinate(initialJointPosition);
}

void RobotVisualization::visualFunction()
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
