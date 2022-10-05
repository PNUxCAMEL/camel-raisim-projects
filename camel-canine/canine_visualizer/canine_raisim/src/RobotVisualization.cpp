//
// Created by camel on 22. 9. 21.
//

#include <canine_raisim/RobotVisualization.hpp>

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

    // base_x,y,z
//    initialJointPosition[0] = sharedMemory->basePosition[0];
//    initialJointPosition[1] = sharedMemory->basePosition[1];
//    initialJointPosition[2] = sharedMemory->basePosition[2];
    initialJointPosition[0] = 0.0;
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 1.0;

    //TODO: sharedMemory->baseEulerPosition[0] to quaternion or other methods are needed
    // base_rotation [quaternion]
    initialJointPosition[3] = 1.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.0;

    // RF_hip,thigh,calf
    initialJointPosition[7] = sharedMemory->motorPosition[RFHR_IDX];
    initialJointPosition[8] = sharedMemory->motorPosition[RFHP_IDX];
    initialJointPosition[9] = sharedMemory->motorPosition[RFKP_IDX];

    // LF_hip,thigh,calf
    initialJointPosition[10] = sharedMemory->motorPosition[LFHR_IDX];
    initialJointPosition[11] = sharedMemory->motorPosition[LFHP_IDX];
    initialJointPosition[12] = sharedMemory->motorPosition[LFKP_IDX];

    // RB_hip,thigh,calf
    initialJointPosition[13] = sharedMemory->motorPosition[RBHR_IDX];
    initialJointPosition[14] = sharedMemory->motorPosition[RBHP_IDX];
    initialJointPosition[15] = sharedMemory->motorPosition[RBKP_IDX];

    // LB_hip,thigh,calf
    initialJointPosition[16] = sharedMemory->motorPosition[LBHR_IDX];
    initialJointPosition[17] = sharedMemory->motorPosition[LBHP_IDX];
    initialJointPosition[18] = sharedMemory->motorPosition[LBKP_IDX];
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
