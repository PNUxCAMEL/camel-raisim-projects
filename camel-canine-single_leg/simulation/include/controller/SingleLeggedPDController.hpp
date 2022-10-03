#ifndef RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
#define RAISIM_SIMPLEPENDULUMPDCONTROLLER_H

#include "Controller.hpp"
#include "camel-tools/trajectory.hpp"
#include "SingleLeggedSharedMemory.hpp"


class SingleLeggedPDController : public Controller
{
public:
    SingleLeggedPDController(Robot* robot, double DT);
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
    double mPGain;
    double mDGain;
    double mTorqueLimit;
    double mDT;
};


#endif //RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
