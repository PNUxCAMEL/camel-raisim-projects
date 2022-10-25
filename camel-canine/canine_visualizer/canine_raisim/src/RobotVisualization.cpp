//
// Created by camel on 22. 9. 21.
//

#include <canine_raisim/RobotVisualization.hpp>

extern pSHM sharedMemory;

RobotVisualization::RobotVisualization(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
    , mTorque(raisim::VecDyn(18))
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(VISUAL_dT);
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
            //openRaisimServer();
            initRobotPose();
            //sharedMemory->visualState = STATE_UPDATE_VISUAL;
            break;
        }
        case STATE_UPDATE_VISUAL:
        {
            if(sharedMemory->simulState == WITH_SIMULATION)
            {
                updateVisualReal();
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

//    // base_x,y,z
//    initialJointPosition[0] = 0.0;
//    initialJointPosition[1] = 0.0;
//    initialJointPosition[2] = 0.37;
//
//    // base_rotation [quaternion]
//    initialJointPosition[3] = 1.0;
//    initialJointPosition[4] = 0.0;
//    initialJointPosition[5] = 0.0;
//    initialJointPosition[6] = 0.0;
//
//    // FR_hip,thigh,calf
//    initialJointPosition[7] = 0.0;
//    initialJointPosition[8] = 0.7;
//    initialJointPosition[9] = -1.4;
//
//    // FL_hip,thigh,calf
//    initialJointPosition[10] = -0.0;
//    initialJointPosition[11] = 0.7;
//    initialJointPosition[12] = -1.4;
//
//    // RR_hip,thigh,calf
//    initialJointPosition[13] = 0.0;
//    initialJointPosition[14] = 0.7;
//    initialJointPosition[15] = -1.4;
//
//    // RL_hip,thigh,calf
//    initialJointPosition[16] = -0.0;
//    initialJointPosition[17] = 0.7;
//    initialJointPosition[18] = -1.4;

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    mRobot->setGeneralizedVelocity(initialJointVelocity);
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