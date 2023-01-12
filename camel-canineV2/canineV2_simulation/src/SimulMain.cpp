//
// Created by hs on 22. 10. 27.
//

#include <canineV2_simulation/SimulMain.hpp>

pthread_t RTThreadControllerHigh;
pthread_t RTThreadControllerLow;
pthread_t RTThreadStateEstimator;
pthread_t NRTThreadCommand;
pthread_t NRTThreadXboxCommand;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/canine/urdf/canineV1.urdf");

SimulCommand userCommand;
//SimulXboxCommand userXboxCommand;
SimulVisualizer Visualizer(&world, robot, &server);
SimulControlPanel ControlPanel(&world, robot);
SimulStateEstimator StateEstimator(robot);
//SimulKalmanFilter StateEstimator(robot);
MPCController MPControl(MPC_HORIZON);

void* NRTCommandThread(void* arg)
{
    std::cout << "entered #Command_NRT_thread" << std::endl;
    while (true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT * 1e6);
    }
}

//void* NRTXboxCommandThread(void* arg)
//{
//    std::cout << "entered #Xbox_Command_NRT_thread" << std::endl;
//    while (true)
//    {
//        userXboxCommand.commandFunction();
//        usleep(CMD_dT * 1e6);
//    }
//}

void *RTControllerThreadHigh(void *arg)
{
    std::cout << "entered #High Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(HIGH_CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            switch (sharedMemory->HighControlState)
            {
                case STATE_CONTROL_STOP:
                {
                    break;
                }
                case STATE_HOME_STAND_UP_READY:
                {
                    MPControl.InitUpTrajectory();
                    sharedMemory->HighControlState = STATE_HOME_CONTROL;
                    sharedMemory->visualState = STATE_UPDATE_VISUAL;
                    break;
                }
                case STATE_HOME_STAND_DOWN_READY:
                {
                    MPControl.InitDownTrajectory();
                    sharedMemory->HighControlState = STATE_HOME_CONTROL;
                    sharedMemory->visualState = STATE_UPDATE_VISUAL;
                    break;
                }
                case STATE_HOME_CONTROL:
                {
                    MPControl.DoControl();
                    sharedMemory->LowControlState = STATE_LOW_CONTROL_START;
                    break;
                }
                default:
                    break;
            }
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, High Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void *RTControllerThreadLow(void *arg)
{
    std::cout << "entered #Low Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(LOW_CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanel.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Low Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}


void* RTStateEstimator(void* arg)
{
    std::cout << "entered #StateEsitimator_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(ESTIMATOR_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            StateEstimator.StateEstimatorFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, State estimator thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->canFLStatus = false;
    sharedMemory->canFRStatus = false;
    sharedMemory->canRLStatus = false;
    sharedMemory->canRRStatus = false;
    sharedMemory->motorStatus = false;
    sharedMemory->HighControlState = STATE_CONTROL_STOP;
    sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->gaitState = STAND;
    sharedMemory->canFLState = CAN_NO_ACT;
    sharedMemory->canFRState = CAN_NO_ACT;
    sharedMemory->canRLState = CAN_NO_ACT;
    sharedMemory->canRRState = CAN_NO_ACT;
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
    sharedMemory->basePosition.setZero();
    sharedMemory->baseVelocity.setZero();
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }

    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;

    sharedMemory->gaitIteration = 0;
}

void StartSimulation()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    server.launchServer(8080);

    int thread_id_rt1 = generate_rt_thread(RTThreadControllerHigh, RTControllerThreadHigh, "rt_thread1", 6, 99, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadControllerLow, RTControllerThreadLow, "rt_thread2", 7, 99, NULL);
    int thread_id_rt3 = generate_rt_thread(RTThreadStateEstimator, RTStateEstimator, "rt_thread3", 8, 99,NULL);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
//    int thread_id_nrt2 = generate_nrt_thread(NRTThreadXboxCommand, NRTXboxCommandThread, "nrt_thread2", 1, NULL);
}