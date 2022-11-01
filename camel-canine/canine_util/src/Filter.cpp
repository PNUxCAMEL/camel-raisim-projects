//
// Created by hs on 22. 11. 1.
//
#include <canine_util/Filter.hpp>

CanineFilter::Vec3LPF::Vec3LPF(double DT, double cutoffFreq)
        : mbIsFirstRun(true)
        , mDT(DT)
        , mCutoffFreq(cutoffFreq)
        , mAlpha(2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1))
{
    mInputData.setZero();
    mPreviousData.setZero();
    mFilteredData.setZero();
}

Vec3<double> CanineFilter::Vec3LPF::GetFilteredVar(const Vec3<double>& data)
{
    for (int idx=0; idx<3; idx++)
    {
        mInputData[idx] = data[idx];
    }
    doFiltering();
    return mFilteredData;
}

void CanineFilter::Vec3LPF::doFiltering()
{
    if (mbIsFirstRun == true)
    {
        for (int idx=0; idx<3; idx++)
        {
            mPreviousData[idx] = mInputData[idx];
        }
        mbIsFirstRun = false;
    }

    for (int idx=0; idx<3; idx++)
    {
        mFilteredData[idx] = mAlpha * mInputData[idx] + (1 - mAlpha) * mPreviousData[idx];
        mPreviousData[idx] = mFilteredData[idx];
    }
}


CanineFilter::LPF::LPF(double DT, double cutoffFreq)
        : mbIsFirstRun(true)
        , mInputData(0.0)
        , mDT(DT)
        , mCutoffFreq(cutoffFreq)
        , mAlpha(2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1))
{
}

double CanineFilter::LPF::GetFilteredVar(const double data)
{
    mInputData = data;
    doFiltering();
    return mFilteredData;
}

void CanineFilter::LPF::doFiltering()
{
    if (mbIsFirstRun == true)
    {
        mPreviousData = mInputData;
        mbIsFirstRun = false;
    }
    mFilteredData = mAlpha * mInputData + (1 - mAlpha) * mPreviousData;
    mPreviousData = mFilteredData;
}