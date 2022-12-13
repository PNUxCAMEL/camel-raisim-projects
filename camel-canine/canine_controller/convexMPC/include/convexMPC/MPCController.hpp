//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include <ControlUtils/SwingLeg.hpp>

#include "MPCSolver.hpp"


using Eigen::Dynamic;

class MPCController{
public:
    MPCController(const uint8_t& horizon);
    void DoControl();
    void InitUpTrajectory();
    void InitDownTrajectory();
private:
    void updateState();
    void computeControlInput();

private:
    const uint8_t mHorizon;
    MPCSolver ConvexMPCSolver;
    CubicTrajectoryGenerator mBaseTrajectory[3];
    CubicTrajectoryGenerator mLegTrajectory[3];

    double mBasePosition[3];
    double mBaseVelocity[3];
    double mBaseEulerPosition[3];
    double mBaseEulerVelocity[3];
    double mFootPosition[4][3];

    Vec3<double> mGRF[4];
    Vec3<double> mTorque[4];
    Vec3<double> mMotorPosition[4];
    Vec3<double> mTorqueJacobian[4];
    Vec13<double> mInitState;
    Mat3<double> mJacobian[4];

};

#endif //RAISIM_MPCCONTROLLER_H
