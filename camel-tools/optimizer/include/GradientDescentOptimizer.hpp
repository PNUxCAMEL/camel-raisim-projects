#ifndef GRADIENTDESCENTOPTIMIZER_H
#define GRADIENTDESCENTOPTIMIZER_H

#include <cmath>
#include <iostream>
#include <Eigen/Eigen>


class GradientDescentOptimizer
{
public:
    GradientDescentOptimizer();

    void SetStepSize(double stepSize);
    void SetMaximumIteration(int maximumIteration);
    void SetTerminateCondition(double terminateCondition);
    void SetObjectiveFunction(double (* objectiveFunction)(Eigen::VectorXd));
    void SetInitialState(Eigen::VectorXd initialState);

    Eigen::VectorXd GetState() const;

    void Solve();

private:
    void computeGradient();
    void updateStates();
    void checkTerminateCondition();

private:
    bool mbTerminateFlag;
    int mIteration;
    int mDimension;
    int mMaximumIteration;
    double mDelta;
    double mStepSize;
    double mRMSGradient;
    double mTerminateCondition;
    double (* mObjectiveFunction)(Eigen::VectorXd);
    Eigen::VectorXd mState;
    Eigen::VectorXd mTempState;
    Eigen::VectorXd mGradient;
};


#endif //GRADIENTDESCENTOPTIMIZER_H
