#ifndef RAISIM_SINGLELEGGEDIDCONTROLLER_H
#define RAISIM_SINGLELEGGEDIDCONTROLLER_H

#include "Controller.hpp"
#include "camel-tools/trajectory.hpp"
#include "SingleLeggedSharedMemory.hpp"

class SingleLeggedIDController : public Controller
{
public:
    SingleLeggedIDController(Robot* robot, double DT);

    void DoControl() override;

private:
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void updateSHM();
    void setPDGain(double const PGain, double const DGain);
    void solveIK();

private:
    Eigen::VectorXd mTorque = Eigen::VectorXd(3);
    Eigen::VectorXd mPositionError = Eigen::VectorXd(2);
    Eigen::VectorXd mVelocityError = Eigen::VectorXd(2);
    Eigen::VectorXd mDesiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd mDesiredJointVelocity = Eigen::VectorXd(2);
    raisim::VecDyn mPosition = raisim::VecDyn(3);
    raisim::VecDyn mVelocity = raisim::VecDyn(3);
    QuinticTrajectoryGenerator mTrajectoryGenerator;
    double mDesiredPosition;
    double mDesiredVelocity;
    double mDesiredAcceleration;
    double mCalculatedForce;
    double mdz_dth1;
    double mdz_dth2;
    double mPGain;
    double mDGain;
    double mTorqueLimit;
    double mLumpedMass;
    double mGravity;
    double mDT;
};

#endif //RAISIM_SINGLELEGGEDIDCONTROLLER_H
