#include "../include/LPF.hpp"

LPF::LPF(double DT, double cutoffFreq)
    : mDT(DT)
    , mCutoffFreq(cutoffFreq)
{
    mInputData = 0.0;
    mbIsFirstRun = true;
    mAlpha = 2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1);
}

double LPF::GetFilteredVar(const double data)
{
    mInputData = data;
    doFiltering();
    return mFilteredData;
}

void LPF::doFiltering()
{
    if (mbIsFirstRun == true)
    {
        mPreviousData = mInputData;
        mbIsFirstRun = false;
    }
    mFilteredData = mAlpha * mInputData + (1 - mAlpha) * mPreviousData;
    mPreviousData = mFilteredData;
}