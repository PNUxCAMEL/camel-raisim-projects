//
// Created by jh on 22. 11. 21.
//

#include <marten-leg_fsm/MainFSM.hpp>

pthread_t RTThreadController;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;
pthread_t NRTThreadCAN;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

CanMotor canMotor("can7");

Command userCommand;
raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/camel_marten_single_leg/marten_single_leg.urdf");
RobotVisualization userVisual(&world, robot, &server);
ControllerState userController;

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

void* NRTCAN(void* arg)
{
    std::cout << "entered #nrt_can_thread" << std::endl;

    while (true)
    {
        canMotor.CanFunction();
    }
}

void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->canStatus = false;
    sharedMemory->motorStatus = false;
    sharedMemory->simulState = true;
    sharedMemory->controlState = STATE_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->canState = CAN_NO_ACT;
    sharedMemory->localTime = 0;
    sharedMemory->desiredHipVerticalPosition = 0;
    sharedMemory->desiredHipVerticalVelocity = 0;
    sharedMemory->hipVerticalPosition = 0;
    sharedMemory->hipVerticalVelocity = 0;
    sharedMemory->hipVerticalAcceleration = 0;
    sharedMemory->estimatedGRFMLP = 0;
    sharedMemory->estimatedGRFSMO = 0;
    sharedMemory->estimatedGRFETO = 0;
    sharedMemory->measuredGRF = 0;
    sharedMemory->desiredGRF = 0;
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

    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        for (int idx = 0; idx < BUF_SIZE; idx++)
        {
            sharedMemory->bufMotorPosition[motorIdx][BUF_SIZE] = 0;
            sharedMemory->bufMotorVelocity[motorIdx][BUF_SIZE] = 0;
        }
    }

    for (int idx = 0; idx < 13; idx++)
    {
        sharedMemory->GRFNetInputs[idx] = 0;
    }
}

void StartFSM()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 6, 99, NULL);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);
    int thread_id_nrt3 = generate_nrt_thread(NRTThreadCAN, NRTCAN, "nrt_thread3", 2, NULL);
}