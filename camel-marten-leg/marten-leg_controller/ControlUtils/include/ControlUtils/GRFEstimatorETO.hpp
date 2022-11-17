//
// Created by cha on 22. 11. 10.
//

#ifndef RAISIM_GRFESTIMATORETO_HPP
#define RAISIM_GRFESTIMATORETO_HPP

#include <Eigen/Eigen>
#include <marten-leg_util/SharedMemory.hpp>
#include <iostream>

class GRFEstimatorETO {
public:
    GRFEstimatorETO();
    void Estimate();
    void UpdateState();
    double GetResidual();

private:
    void updateBeta();
    void updateCoriMat();
    void updateMassMat();


private:
    bool mbIsFirstRun;
    double mBeta;
    double mCoriMat;
    double mGain;
    double mLink1;
    double mLink2;
    double mMass1;
    double mMass2;
    double mMassMat;
    double mMomentum;
    double mMomentumPrev;
    double mResidual;
};


#endif //RAISIM_GRFESTIMATORETO_HPP
