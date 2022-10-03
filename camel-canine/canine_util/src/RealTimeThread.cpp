//
// Created by hs on 22. 10. 3.
//

#include <canine_util/RealTimeThread.hpp>

int RealTimeThread::generate_rt_thread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg){
    struct sched_param param;
    pthread_attr_t attr;
    cpu_set_t cpuset;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        printf("init pthread attributes failed\n");
        return false;
    }

//#ifdef EXTERNAL
//    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
//    if(ret){
//        printf("pthread setstacksize failed\n");
//        return false;
//    }
//#endif

    if (cpu_no >= 0) {
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_no, &cpuset);
        ret = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if(ret){
            printf("pthread cpu allocation failed\n");
            return false;
        }
    }

#ifdef EXTERNAL
    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if(ret){
        printf("pthread setschedpolicy failed\n");
        return false;
    }
    param.sched_priority = priority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if(ret){
        printf("pthread setschedparam failed\n");
        return false;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if(ret){
        printf("pthread setinheritsched failed\n");
        return false;
    }
#endif

    ret = pthread_create(&thread_rt, &attr, thread_func, arg);
    if(ret){
        printf("create pthread failed\n");
        return false;
    }

    ret = pthread_setname_np(thread_rt, name);
    if(ret){
        printf("thread naming failed\n");
        return false;
    }
    return true;
}