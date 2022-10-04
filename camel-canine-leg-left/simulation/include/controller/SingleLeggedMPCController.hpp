#ifndef RAISIM_SINGLELEGGEDMPCCONTROLLER_H
#define RAISIM_SINGLELEGGEDMPCCONTROLLER_H

#include "Controller.hpp"
#include "camel-tools/trajectory.hpp"
#include "SingleLeggedSharedMemory.hpp"

class SingleLeggedMPCController : public Controller
{
public:
    SingleLeggedMPCController(Robot* robot, double DT, int MPCHorizon);

    void DoControl() override;

private:
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void updateSHM();
    void solve();
    void updateMPCStates();
    void updateMPCStatesTemp(Eigen::VectorXd const force);
    void computeGradient();
    void updateForces();
    void resetMPCVariables();
    bool isTerminateCondition();
    double objectiveFunction(Eigen::VectorXd const force);

private:
    Eigen::VectorXd mTorque = Eigen::VectorXd(3);
    raisim::VecDyn mPosition = raisim::VecDyn(3);
    raisim::VecDyn mVelocity = raisim::VecDyn(3);
    SineTrajectoryGenerator mTrajectoryGenerator;
    double mCalculatedForce;
    double mdz_dth1;
    double mdz_dth2;
    double mTorqueLimit;
    double mLumpedMass;
    double mGravity;
    double mDT;

    int mMPCHorizon;
    int mIteration;
    int mMaximumIteration;
    double mTerminateCondition;
    double mDelta;
    double mStepSize;
    double mInitialPosition;
    double mInitialVelocity;
    double mRMSGradient;
    double mObjFunctionValue;
    double tempValue1;
    double tempValue2;
    Eigen::MatrixXd mTrajectorySequence = Eigen::MatrixXd(2, mMPCHorizon);
    Eigen::MatrixXd mA = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mB = Eigen::VectorXd(2);
    Eigen::VectorXd mC = Eigen::VectorXd(2);
    Eigen::MatrixXd mQ = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mR = Eigen::VectorXd(1);
    Eigen::VectorXd mGradient = Eigen::VectorXd(mMPCHorizon);
    Eigen::MatrixXd mNextStates = Eigen::MatrixXd(2, mMPCHorizon);
    Eigen::MatrixXd mNextStatesTemp = Eigen::MatrixXd(2, mMPCHorizon);
    Eigen::VectorXd mForce = Eigen::VectorXd(mMPCHorizon);
    Eigen::VectorXd mForceTemp = Eigen::VectorXd(mMPCHorizon);
    Eigen::VectorXd mNextX = Eigen::VectorXd(2);
    Eigen::VectorXd mNextXDes = Eigen::VectorXd(2);
};


#endif //RAISIM_SINGLELEGGEDMPCCONTROLLER_H
