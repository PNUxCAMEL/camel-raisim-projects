//
// Created by hs on 22. 10. 3.
//

#ifndef RAISIM_THREADFUNCTION_HPP
#define RAISIM_THREADFUNCTION_HPP

#include <pthread.h>

class ThreadFunction{
public:
    void timespec_add_us(struct timespec *t, long us);
    int timespec_cmp(struct timespec *a, struct timespec *b);
    int timediff_us(struct timespec *before, struct timespec *after);
};

#endif //RAISIM_THREADFUNCTION_HPP
