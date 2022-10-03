#ifndef LPF_HPP
#define LPF_HPP

#include <iostream>

class LPF
{
public:
    LPF(double dt, double fc);

    double GetFilteredVar(double const data);

private:
    void doFiltering();

private:
    bool mbIsFirstRun;
    double mDT;
    double mCutoffFreq;
    double mAlpha;
    double mInputData;
    double mPreviousData;
    double mFilteredData;

};


#endif //LPF_HPP
