#include <iostream>
#include <unistd.h>
#include "camel-tools/ThreadGenerator.hpp"

pthread_t thread_rt;

void* rt_thread(void* arg)
{
    std::cout << "entered #rt_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;
    struct timespec TIME_TOC;
    struct timespec TIME_START;
    struct timespec TIME_END;
    int iteration = 0;
    int failedTime = 0;
    unsigned long mpc_counter = 0;
    const long PERIOD_US = 1000; // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    clock_gettime(CLOCK_REALTIME, &TIME_START);

    std::cout << "bf #while" << std::endl;
    while (1)
    {

        iteration++;

        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, thread :  " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
            failedTime++;
        }
        if (iteration == 10000)
        {
            break;
        }
    }
    clock_gettime(CLOCK_REALTIME, &TIME_END);
    float intervalTime_ms = timediff_us(&TIME_START, &TIME_END) * 0.001;
    std::cout << "interval[ms] : " << intervalTime_ms << std::endl;
    std::cout << "failed num : " << failedTime << std::endl;
}

int main()
{

    std::cout << "bf #generate_rt_thread" << std::endl;
    int thread_id_rt = generate_rt_thread_hard(thread_rt, rt_thread, "rt_thread", 0, 10, NULL);

    sleep(3000);
    return 0;
}