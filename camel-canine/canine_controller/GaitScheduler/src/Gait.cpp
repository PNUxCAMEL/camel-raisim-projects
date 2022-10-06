//
// Created by hs on 22. 8. 10.
//

#include "../include/GaitScheduler/Gait.hpp"

OffsetGait::OffsetGait(int mpcHorizon, Vec4<int> offsets, Vec4<int> durations, int cyclePeriod)
    : mOffsets(offsets.array())
    , mDurations(durations.array())
    , mHorizon(mpcHorizon)
    , mCyclePeriod(cyclePeriod)
    , mGaitTable(new int[mpcHorizon*4])
{
}

OffsetGait::~OffsetGait() noexcept {
    delete[] mGaitTable;
}

int *OffsetGait::getGaitTable() {
    //printf("MPC table:\n");
    for(int i = 0; i < mHorizon; i++)
    {
        int iter = (i + mIteration) % mCyclePeriod;
        Eigen::Array4i progress = iter - mOffsets;
        for(int j = 0; j < 4; j++)
        {
            if(progress[j] < 0) progress[j] += mCyclePeriod;
            if(progress[j] < mDurations[j])
                mGaitTable[i*4 + j] = 1;
            else
                mGaitTable[i*4 + j] = 0;

            //printf("%d ", _gaitTable[i*4 + j]);
        }
        //printf("\n");
    }
    return mGaitTable;
}

void OffsetGait::setIterations(int iteration) {
    mIteration = iteration % mCyclePeriod;
}