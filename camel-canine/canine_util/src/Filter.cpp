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
    std::cout << data << std::endl;
    for (uint8_t idx=0; idx<3; idx++)
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
        for (uint8_t idx=0; idx<3; idx++)
        {
            mPreviousData[idx] = mInputData[idx];
        }
        mbIsFirstRun = false;
    }

    for (uint8_t idx=0; idx<3; idx++)
    {
        mFilteredData[idx] = mAlpha * mInputData[idx] + (1 - mAlpha) * mPreviousData[idx];
        mPreviousData[idx] = mFilteredData[idx];
    }
}