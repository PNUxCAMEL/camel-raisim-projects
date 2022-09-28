//
// Created by hs on 22. 8. 10.
//

#ifndef RAISIM_GAIT_H
#define RAISIM_GAIT_H

#include "EigenTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;

enum class GaitType {
    STAND,
    TROT,
    PACE,
    BOUND
};

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
    int* _gaitTable;
    int _iteration;
    int _nHorizon;
    int _cyclePeriod;
    Array4i _offsets;
    Array4i _durations;
};



#endif //RAISIM_GAIT_H
