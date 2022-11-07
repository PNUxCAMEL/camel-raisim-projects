//
// Created by jh on 22. 11. 7.
//

#ifndef RAISIM_MPCCONTROLLER_HPP
#define RAISIM_MPCCONTROLLER_HPP

#include <Eigen/Eigen>
#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>
#include <camel-tools/SineTrajectoryGenerator.hpp>

class MPCcontroller
{
public:
    MPCcontroller();

    void DoControl();
    void InitSineTrajectory();
    void SetControlInput();

private:
    void setSineTrajectory();
    void computeControlInput();
    void solve();
    void updateMPCStates();
    void updateMPCStatesTemp(Eigen::VectorXd const force);
    void computeGradient();
    void updateForces();
    void resetMPCVariables();
    bool isTerminateCondition();
    double objectiveFunction(Eigen::VectorXd const force);

private:
    SineTrajectoryGenerator mSineTrajectoryGenerator;
    Eigen::MatrixXd mTrajectorySequence = Eigen::MatrixXd(MOTOR_NUM, MPC_HORIZON);
    Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mTorque = Eigen::VectorXd(MOTOR_NUM);
    Eigen::MatrixXd mA = Eigen::MatrixXd(2, 2);

    Eigen::VectorXd mB = Eigen::VectorXd(2);
    Eigen::VectorXd mC = Eigen::VectorXd(2);
    Eigen::MatrixXd mQ = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mR = Eigen::VectorXd(1);
    Eigen::VectorXd mGradient = Eigen::VectorXd(MPC_HORIZON);
    Eigen::MatrixXd mNextStates = Eigen::MatrixXd(MOTOR_NUM, MPC_HORIZON);
    Eigen::MatrixXd mNextStatesTemp = Eigen::MatrixXd(MOTOR_NUM, MPC_HORIZON);
    Eigen::VectorXd mForce = Eigen::VectorXd(MPC_HORIZON);
    Eigen::VectorXd mForceTemp = Eigen::VectorXd(MPC_HORIZON);
    Eigen::VectorXd mNextX = Eigen::VectorXd(MOTOR_NUM);
    Eigen::VectorXd mNextXDes = Eigen::VectorXd(MOTOR_NUM);

    double mTorqueLimit[MOTOR_NUM];
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
};


#endif //RAISIM_MPCCONTROLLER_HPP
