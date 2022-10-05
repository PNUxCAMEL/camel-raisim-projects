//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include "qpsolver.hpp"
#include "Trajectory.hpp"

using Eigen::Dynamic;

class MPCController{
public:
    //Constructor
    MPCController(raisim::World *world, raisim::ArticulatedSystem *robot, double dT);

    void doControl();
    void setTrajectory();

    void setGait(int index);
    void resetParam(int, double);
    void resetWeight(Vec13<double>, double);

    // For real-time plotting
    ConvexMPCSolver cmpcSolver;
    raisim::VecDyn position = raisim::VecDyn(19);

private:
    void initialize();
    void updateState();
    void setLegcontrol();
    void computeControlInput();
    void setControlInput();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;
    double mDT;

    int *mpcTable;
    int mMPCHorizon;

    Gait *currentGait;
    GaitType currentGaitName;

    OffsetGait stand, trot, pace, bound;

    int iteration = 0;

    LegTrajectory legGenerator;

    double alpha = 1e-10;
    double torqueLimit = 50.0;
    double legDpos = 0.0;

    raisim::VecDyn velocity = raisim::VecDyn(18);
    raisim::VecDyn torque = raisim::VecDyn(18);
    raisim::VecDyn Legtemptorque = raisim::VecDyn(12);

    raisim::Mat<3, 3> bdyInertia;
    raisim::Vec<3> footPosition[4];

    Eigen::Matrix<double,3,1> f[4], robottorque[4];
    Eigen::Matrix<double,3,3> robotJacobian[4];
    Eigen::Matrix<double, 13,1> weightMat;
};

#endif //RAISIM_MPCCONTROLLER_H
