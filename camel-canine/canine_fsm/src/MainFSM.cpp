//
// Created by hs on 22. 10. 5.
//

#include <canine_fsm/MainFSM.hpp>

pthread_t RTThreadController;
pthread_t RTThreadStateEstimator;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;
pthread_t NRTThreadIMU;
pthread_t NRTThreadCANForward;
pthread_t NRTThreadCANBackward;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

CANMotorForward canForward("can9");
CanMotorBackward canBackward("can5");

Command userCommand;

raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/canine/urdf/canineV1.urdf");
RobotVisualization userVisual(&world, robot, &server);
StateEstimator robotstate;
ControllerState userController;

const std::string mComPort = "/dev/ttyACM0";
const mscl::Connection mConnection = mscl::Connection::Serial(mComPort);
mscl::InertialNode node(mConnection);
LordImu3DmGx5Ahrs IMUBase(&node);



void* NRTCommandThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while (true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT * 1e6);
    }
}

void* NRTVisualThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while (true)
    {
        userVisual.VisualFunction();
        usleep(VISUAL_dT * 1e6);
    }
}

//TODO: Cha is gonna implemet "GetLinearVelocity" and "GetEulerVelocity" from T265 tracking camera.
void* NRTImuThread(void* arg)
{
    double Offset[3]={0.0,0.0,0.0};
    std::cout << "entered #nrt_IMU_thread" << std::endl;
    IMUBase.SetConfig(250);
    double* baseAngularPosition;
    double* baseAngularVelocity;

    mscl::EulerAngles IMUAngularPositionOffset(3.141592, 0, 0);
    node.setSensorToVehicleRotation_eulerAngles(IMUAngularPositionOffset);

    for (int idx=0; idx<500; idx++)
    {
        IMUBase.ParseData();
        baseAngularPosition = IMUBase.GetEulerAngle();
        Offset[0] += baseAngularPosition[0];
        Offset[1] += baseAngularPosition[1];
        Offset[2] += baseAngularPosition[2];
    }
    Offset[0] /= 500;
    Offset[1] /= 500;
    Offset[2] /= 500;

    while (true)
    {
        IMUBase.ParseData();

        baseAngularPosition = IMUBase.GetEulerAngle();
        baseAngularVelocity = IMUBase.GetAngularVelocity();

        sharedMemory->baseEulerPosition[0] =   baseAngularPosition[0]-Offset[0];
        sharedMemory->baseEulerPosition[1] = -(baseAngularPosition[1]-Offset[1]);
        sharedMemory->baseEulerPosition[2] = -(baseAngularPosition[2]-Offset[2]);

        sharedMemory->baseEulerVelocity[0] = baseAngularVelocity[0];
        sharedMemory->baseEulerVelocity[1] = -baseAngularVelocity[1];
        sharedMemory->baseEulerVelocity[2] = -baseAngularVelocity[2];

        double cy = cos(sharedMemory->baseEulerPosition[2] * 0.5);
        double sy = sin(sharedMemory->baseEulerPosition[2] * 0.5);
        double cp = cos(sharedMemory->baseEulerPosition[1] * 0.5);
        double sp = sin(sharedMemory->baseEulerPosition[1] * 0.5);
        double cr = cos(sharedMemory->baseEulerPosition[0] * 0.5);
        double sr = sin(sharedMemory->baseEulerPosition[0] * 0.5);

        sharedMemory->baseQuartPosition[0] = cr * cp * cy + sr * sp * sy;
        sharedMemory->baseQuartPosition[1] = sr * cp * cy - cr * sp * sy;
        sharedMemory->baseQuartPosition[2] = cr * sp * cy + sr * cp * sy;
        sharedMemory->baseQuartPosition[3] = cr * cp * sy - sr * sp * cy;

        usleep(IMU_dT * 1e6);
    }
}

void* RTControllerThread(void* arg)
{
    std::cout << "entered #rt_controller_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CONTROL_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << "Hz" << std::endl;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        userController.ControllerFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* RTStateEstimator(void* arg)
{
    std::cout << "entered #rt_state_estimation_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(ESTIMATOR_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        robotstate.StateEstimatorFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, state estimation thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}


void* NRTCANForward(void* arg)
{
    std::cout << "entered #nrt_can_forward_thread" << std::endl;

    while (true)
    {
        canForward.CanFunction();
    }
}

void* NRTCANBackward(void* arg)
{
    std::cout << "entered #nrt_can_backward_thread" << std::endl;
    while (true)
    {
        canBackward.CanFunction();
    }
}

//TODO: add other variables
void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->can1Status = false;
    sharedMemory->can2Status = false;
    sharedMemory->motorStatus = false;
    sharedMemory->controlState = STATE_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->can1State = CAN_NO_ACT;
    sharedMemory->can2State = CAN_NO_ACT;
    sharedMemory->localTime = 0;
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorVoltage[index] = 0;
    }
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->basePosition[index] = 0;
        sharedMemory->baseVelocity[index] = 0;
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }

    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;

    sharedMemory->gaitState = STAND;
    sharedMemory->gaitIteration = 0;
    sharedMemory->motorForeState = false;
    sharedMemory->motorBackState = false;
}

void StartFSM()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 6, 99, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadStateEstimator, RTStateEstimator, "rt_thread2", 7, 99,NULL);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);
    int thread_id_nrt3 = generate_nrt_thread(NRTThreadIMU, NRTImuThread, "nrt_thread3", 2, NULL);
    int thread_id_nrt4 = generate_nrt_thread(NRTThreadCANForward, NRTCANForward, "nrt_thread4", 3, NULL);
    int thread_id_nrt5 = generate_nrt_thread(NRTThreadCANBackward, NRTCANBackward, "nrt_thread5", 4, NULL);

}