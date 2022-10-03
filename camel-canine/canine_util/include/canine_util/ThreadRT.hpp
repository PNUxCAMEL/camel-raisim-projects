//
// Created by hs on 22. 10. 3.
//

#ifndef RAISIM_THREADRT_HPP
#define RAISIM_THREADRT_HPP

#include <stdio.h>

#include <pthread.h>

class ThreadRT{
public:
    int generate_rt_thread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg);
};

#endif //RAISIM_THREADRT_HPP
