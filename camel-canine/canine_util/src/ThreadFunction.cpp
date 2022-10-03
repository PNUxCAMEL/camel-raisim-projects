//
// Created by hs on 22. 10. 3.
//

#include <canine_util/ThreadFunction.hpp>

void ThreadFunction::timespec_add_us(struct timespec *t, long us){
    t->tv_nsec += us*1000;
    if(t->tv_nsec > 1000000000){
        t->tv_nsec = t->tv_nsec - 1000000000;
        t->tv_sec += 1;
    }
}
int ThreadFunction::timespec_cmp(struct timespec *a, struct timespec *b){
    if(a->tv_sec > b->tv_sec) return 1;
    else if(a->tv_sec < b->tv_sec) return -1;
    else if(a->tv_sec == b->tv_sec){
        if(a->tv_nsec > b->tv_nsec) return 1;
        else if(a->tv_nsec == b->tv_nsec) return 0;
        else return -1;
    }
}
int ThreadFunction::timediff_us(struct timespec *before, struct timespec *after){
    return (after->tv_sec - before->tv_sec)*1e6 + (after->tv_nsec - before->tv_nsec)/1000;
}
