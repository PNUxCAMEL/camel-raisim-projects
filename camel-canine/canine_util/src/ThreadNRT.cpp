//
// Created by hs on 22. 10. 3.
//

#include <canine_util/ThreadNRT.hpp>

int ThreadNRT::generate_nrt_thread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpu_no, void *arg){
    pthread_attr_t attr;
    cpu_set_t cpuset;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        printf("init pthread attributes failed\n");
        return false;
    }

    if (cpu_no >= 0) {
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_no, &cpuset);
        ret = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if(ret){
            printf("pthread cpu allocation failed\n");
            return false;
        }
    }

    ret = pthread_create(&thread_nrt, &attr, thread_func, arg);
    if(ret){
        printf("create pthread failed\n");
        return false;
    }

    ret = pthread_setname_np(thread_nrt, name);
    if(ret){
        printf("thread naming failed\n");
        return false;
    }

    return false;
}