#include "../include/ThreadGenerator.hpp"

int generate_nrt_thread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpu_no, void *arg){
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
int generate_rt_thread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg){
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

int generate_rt_thread_hard(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg){
    struct sched_param param;
    pthread_attr_t attr;
    cpu_set_t cpuset;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        printf("init pthread attributes failed\n");
        return false;
    }

#ifdef EXTERNAL
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if(ret){
        printf("pthread setstacksize failed\n");
        return false;
    }
#endif

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

void timespec_add_us(struct timespec *t, long us){
    t->tv_nsec += us*1000;
    if(t->tv_nsec > 1000000000){
        t->tv_nsec = t->tv_nsec - 1000000000;
        t->tv_sec += 1;
    }
}
int timespec_cmp(struct timespec *a, struct timespec *b){
    if(a->tv_sec > b->tv_sec) return 1;
    else if(a->tv_sec < b->tv_sec) return -1;
    else if(a->tv_sec == b->tv_sec){
        if(a->tv_nsec > b->tv_nsec) return 1;
        else if(a->tv_nsec == b->tv_nsec) return 0;
        else return -1;
    }
}
int timediff_us(struct timespec *before, struct timespec *after){
    return (after->tv_sec - before->tv_sec)*1e6 + (after->tv_nsec - before->tv_nsec)/1000;
}
