//
// Created by camel on 22. 9. 21.
//

#include <canineV2_raisim/RobotVisualization.hpp>

extern pSHM sharedMemory;

RobotVisualization::RobotVisualization(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(VISUAL_dT);
    mWorld->addGround();
    mRobot->setName("Canine");
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
    initialJointPosition[0] = 0.0;
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 1.0;

    //TODO: sharedMemory->baseEulerPosition[0] to quaternion or other methods are needed
    // base_rotation [quaternion]
    initialJointPosition[3] = sharedMemory->baseQuartPosition[0];
    initialJointPosition[4] = sharedMemory->baseQuartPosition[1];
    initialJointPosition[5] = sharedMemory->baseQuartPosition[2];
    initialJointPosition[6] = sharedMemory->baseQuartPosition[3];

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
