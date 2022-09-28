//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_MPCCONTROLLER_H
#define RAISIM_MPCCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "src/CANINE/canine/CanineRobot.h"
#include "include/Xbox/XboxController.h"

#include "Gait.h"
#include "qpsolver.h"
#include "SwingLegTrajectory.h"
#include "EigenTypes.h"

using Eigen::Dynamic;

class MPCController : public Controller {
public:
    //Constructor
    MPCController(Robot *robot, double dT);

    void initialize();
    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

    void setLegcontrol();
    void setGait(int index);
    void resetParam(int, double);
    void resetWeight(Vec13<double>, double);

    // For real-time plotting
    ConvexMPCSolver cmpcSolver;
    raisim::VecDyn position = raisim::VecDyn(19);
    raisim::Vec<3> footPosition[4];

private:
    XboxController joystick = XboxController();
    int *mpcTable;
    int mMPCHorizon;
    double mDT;
    Gait *currentGait;
    GaitType currentGaitName;

    OffsetGait stand, trot, pace, bound;

    int iteration = 0;

    SwingLegTrajectory SwinglegGenerator;

    double alpha = 1e-10;
    double torqueLimit = 50.0;
    double desiredPosition[3] = {0.0, 0.0, 0.0};


    raisim::VecDyn velocity = raisim::VecDyn(18);
    raisim::VecDyn torque = raisim::VecDyn(18);
    raisim::VecDyn Legtemptorque = raisim::VecDyn(12);

    raisim::Mat<3, 3> bdyInertia;

    Eigen::Matrix<double,3,1> f[4], robottorque[4];
    Eigen::Matrix<double,3,3> robotJacobian[4];
    Eigen::Matrix<double, 13,1> weightMat;
};

#endif //RAISIM_MPCCONTROLLER_H
