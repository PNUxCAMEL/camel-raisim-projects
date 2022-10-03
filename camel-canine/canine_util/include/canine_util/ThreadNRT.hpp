//
// Created by hs on 22. 10. 3.
//

#ifndef RAISIM_THREADNRT_HPP
#define RAISIM_THREADNRT_HPP

#include <stdio.h>

#include <pthread.h>

class ThreadNRT{
public:
    int generate_nrt_thread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpu_no, void *arg);
};


#endif //RAISIM_THREADNRT_HPP
