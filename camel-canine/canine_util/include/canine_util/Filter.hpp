//
// Created by hs on 22. 11. 1.
//

#ifndef RAISIM_FILTER_HPP
#define RAISIM_FILTER_HPP

#include <iostream>
#include <canine_util/EigenTypes.hpp>

namespace CanineFilter
{
    class Vec3LPF
    {
    public:
        Vec3LPF(double DT, double cutoffFreq);
        Vec3<double> GetFilteredVar(const Vec3<double>& data);
    private:
        void doFiltering();
    private:
        bool mbIsFirstRun;
        double mDT;
        double mCutoffFreq;
        double mAlpha;
        Vec3<double> mInputData;
        Vec3<double> mPreviousData;
        Vec3<double> mFilteredData;
    };

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
}



#endif //RAISIM_FILTER_HPP
