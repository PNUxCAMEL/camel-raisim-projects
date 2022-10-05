//
// Created by hs on 22. 8. 21.
//

#include "camel-canine/CanineMain.hpp"

void *NRTCommandThread(void *arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while(true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT*1e6);
    }
}

void *NRTVisualThread(void *arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while(true)
    {
        userVisual.visualFunction();
        usleep(VISUAL_dT*1e6);
    }
}

void *RTControllerThread(void *arg) {
    std::cout << "entered #rt_controller_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CONTROL_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 <<"Hz"<< std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        userController.controllerFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

//TODO: add other variables
void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->canStatus = false;
    sharedMemory->motorStatus = false;
    sharedMemory->controlState = STATE_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
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
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    sharedCommand = (pUI_COMMAND) malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    clearSharedMemory();

    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 6, 99,NULL);
    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);

    w.show();
    return a.exec();
}