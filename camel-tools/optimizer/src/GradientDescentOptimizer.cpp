#include "../include/GradientDescentOptimizer.hpp"

GradientDescentOptimizer::GradientDescentOptimizer()
    : mbTerminateFlag(false)
    , mIteration(0)
    , mDimension(0)
    , mMaximumIteration(1000)
    , mDelta(1e-6)
    , mStepSize(0.01)
    , mRMSGradient(0.0)
    , mTerminateCondition(1e-6)
{
}

void GradientDescentOptimizer::SetStepSize(double stepSize)
{
    mStepSize = stepSize;
}

void GradientDescentOptimizer::SetMaximumIteration(int maximumIteration)
{
    mMaximumIteration = maximumIteration;
}

void GradientDescentOptimizer::SetTerminateCondition(double terminateCondition)
{
    mTerminateCondition = terminateCondition;
}

void GradientDescentOptimizer::SetObjectiveFunction(double (* objectiveFunction)(Eigen::VectorXd))
{
    mObjectiveFunction = objectiveFunction;
}

void GradientDescentOptimizer::SetInitialState(Eigen::VectorXd initialState)
{
    mState = initialState;
    mDimension = mState.size();
    mTempState = Eigen::VectorXd(mDimension);
    mTempState.setZero();
    mGradient = Eigen::VectorXd(mDimension);
    mGradient.setZero();
}

Eigen::VectorXd GradientDescentOptimizer::GetState() const
{
    return mState;
}

void GradientDescentOptimizer::computeGradient()
{
    double functionValue = mObjectiveFunction(mState);
    for (int i = 0; i < mDimension; i++)
    {
        mTempState = mState;
        mTempState[i] += mDelta;
        mGradient[i] = (mObjectiveFunction(mTempState) - functionValue) / mDelta;
    }
    mRMSGradient = pow(mGradient.dot(mGradient) / mDimension, 0.5);
}

void GradientDescentOptimizer::updateStates()
{
    mState = mState - mStepSize * mGradient;
}

void GradientDescentOptimizer::checkTerminateCondition()
{
    if (mIteration == mMaximumIteration)
    {
        mbTerminateFlag = true;
        std::cout << "Gradient Descent Optimizer is completed. (Maximum iteration)" << std::endl;
    }
    else if (mRMSGradient < mTerminateCondition)
    {
        mbTerminateFlag = true;
        std::cout << "Gradient Descent Optimizer is completed. (Terminate condition)" << std::endl;
    }
}

void GradientDescentOptimizer::Solve()
{
    for (int i = 0; i < mMaximumIteration; i++)
    {
        mIteration++;
        computeGradient();
        updateStates();
        checkTerminateCondition();
        if (mbTerminateFlag)
        {
            break;
        }
    }
}