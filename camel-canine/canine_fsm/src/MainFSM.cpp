//
// Created by hs on 22. 10. 5.
//

#include <canine_fsm/MainFSM.hpp>

pthread_t RTThreadController;
pthread_t RTThreadCANForward;
pthread_t RTThreadCANBackward;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

CANMotorForward canForward("can9");
CanMotorBackward canBackward("can5");

Command userCommand;

ControllerState userController;

raisim::World world;
raisim::RaisimServer server(&world);
RobotVisualization userVisual(&world, &server);


void* NRTCommandThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while(true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT*1e6);
    }
}

void* NRTVisualThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while(true)
    {
        userVisual.VisualFunction();
        usleep(VISUAL_dT*1e6);
    }
}

void* RTControllerThread(void* arg) {
    std::cout << "entered #rt_controller_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CONTROL_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 <<"Hz"<< std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        userController.ControllerFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

void* RTCANForward(void* arg) {
    std::cout << "entered #rt_can_forward_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CAN_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        canForward.CanFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, can forward thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                << " ms" << std::endl;
        }
    }
}

void* RTCANBackward(void* arg) {
    std::cout << "entered #rt_can_backward_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CAN_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        canBackward.CanFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, can backward thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                << " ms" << std::endl;
        }
    }
}

//TODO: add other variables
void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->can1Status = false;
    sharedMemory->can2Status = false;
    sharedMemory->motorStatus = false;
    sharedMemory->simulState = true;
    sharedMemory->controlState = STATE_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->can1State = CAN_NO_ACT;
    sharedMemory->can2State = CAN_NO_ACT;
    sharedMemory->localTime = 0;
    for(int index = 0; index < MOTOR_NUM ; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorVoltage[index] = 0;
    }
    for(int index = 0; index < 3 ; index++)
    {
        sharedMemory->basePosition[index] = 0;
        sharedMemory->baseVelocity[index] = 0;
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }
}

void StartFSM()
{
    sharedCommand = (pUI_COMMAND) malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    clearSharedMemory();

    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 5, 99,NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadCANForward, RTCANForward, "rt_thread2", 6, 99,NULL);
    int thread_id_rt3 = generate_rt_thread(RTThreadCANBackward, RTCANBackward, "rt_thread3", 7, 99,NULL);
    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);

}