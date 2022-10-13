//
// Created by camel on 22. 9. 21.
//

#include <canine_raisim/RobotVisualization.hpp>

extern pSHM sharedMemory;

RobotVisualization::RobotVisualization(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(0.01);
    mWorld->addGround();
    mRobot->setName("Canine");
    mTorque.setZero();
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
            initRobotPose();
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            break;
        }
        case STATE_UPDATE_VISUAL:
        {
            if(sharedMemory->simulState == WITH_SIMULATION)
            {
                updateVisualReal();
            }
            else
            {

                updateVisualSimul();
            }
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

void RobotVisualization::initRobotPose()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();

    initialJointPosition[2] = 0.0578;
    initialJointPosition[3] = 1.0;

    for (int idx=0; idx<4; idx++)
    {
        initialJointPosition[idx*3+7] = 0.0;
        initialJointPosition[idx*3+8] = 2.5;
        initialJointPosition[idx*3+9] = -2.9;
    }
    mRobot->setGeneralizedCoordinate(initialJointPosition);
}

void RobotVisualization::updateVisualReal()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();

    // base_x,y,z
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

void RobotVisualization::updateVisualSimul()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        mTorque[idx+6] = sharedMemory->motorDesiredTorque[idx];
    }
    mRobot->setGeneralizedForce(mTorque);
    mServer->integrateWorldThreadSafe();
}