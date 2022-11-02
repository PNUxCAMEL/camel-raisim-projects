//
// Created by jh on 22. 11. 3.
//

#ifndef RAISIM_GRFESTIMATOR_HPP
#define RAISIM_GRFESTIMATOR_HPP

#include <array>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "tensorflow/c/c_api.h"
#include <camel-tools/ThreadGenerator.hpp>
#include <canine-leg-left_util/SharedMemory.hpp>

class GRFEstimator
{
public:
    GRFEstimator();
    void Estimate();

private:
    TF_Buffer* mRunOption;
    TF_SessionOptions* mSessionOptions;
    TF_Graph* mGraph;
    TF_Status* mStatus;
    TF_Session* mSession;
    TF_Operation* mInputOperation;
    TF_Operation* mOutputOperation;
    std::array<TF_Output, 1> mInputOps;
    std::array<TF_Output, 1> mOutputOps;

};


#endif //RAISIM_GRFESTIMATOR_HPP
