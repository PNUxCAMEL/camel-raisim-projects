//
// Created by hs on 22. 8. 10.
//
#include
#include "Gait.h"
#include <iostream>

/**
 *
 * @param mpcHorizon
 * @param offsets
 * @param durations
 * @param cyclePeriod  1Cycle = dt * cyclePeriod
 */
OffsetGait::OffsetGait(int mpcHorizon, Vec4<int> offsets, Vec4<int> durations, int cyclePeriod) :
_offsets(offsets.array()),
_durations(durations.array()),
_nHorizon(mpcHorizon),
_cyclePeriod(cyclePeriod)
{
    _gaitTable = new int[mpcHorizon*4];
}

OffsetGait::~OffsetGait() noexcept {
    delete[] _gaitTable;
}

int *OffsetGait::getGaitTable() {
    //printf("MPC table:\n");
    for(int i = 0; i < _nHorizon; i++)
    {
        int iter = (i + _iteration) % _cyclePeriod;
        Array4i progress = iter - _offsets;
        for(int j = 0; j < 4; j++)
        {
            if(progress[j] < 0) progress[j] += _cyclePeriod;
            if(progress[j] < _durations[j])
                _gaitTable[i*4 + j] = 1;
            else
                _gaitTable[i*4 + j] = 0;

            //printf("%d ", _gaitTable[i*4 + j]);
        }
        //printf("\n");
    }
    return _gaitTable;
}

void OffsetGait::setIterations(int iteration) {
    _iteration = iteration % _cyclePeriod;
}