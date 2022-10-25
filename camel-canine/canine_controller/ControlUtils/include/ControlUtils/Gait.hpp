//
// Created by hs on 22. 8. 10.
//

#ifndef RAISIM_GAIT_H
#define RAISIM_GAIT_H

#include <iostream>
#include <canine_util/EigenTypes.hpp>

class Gait{
public:
    virtual int* getGaitTable() = 0;
    virtual void setIterations(int iteration) = 0;
};

class OffsetGait : public Gait {
public:
    OffsetGait(int mpcHorizon, Vec4<int> offsets, Vec4<int> durations, int cyclePeriod);
    ~OffsetGait();
    int* getGaitTable() override;
    void setIterations(int iteration) override;

private:
    int* mGaitTable;
    int mIteration;
    int mHorizon;
    int mCyclePeriod;
    Eigen::Array4i mOffsets;
    Eigen::Array4i mDurations;
};



#endif //RAISIM_GAIT_H
