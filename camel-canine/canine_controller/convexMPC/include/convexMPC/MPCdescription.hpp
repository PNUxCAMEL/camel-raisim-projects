//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_MPCDESCRIPTION_HPP
#define RAISIM_MPCDESCRIPTION_HPP

#include <iostream>
#include <Eigen/Eigen>

#define BIG_NUMBER 5e10
#define GRAVITY -9.81
#define BODYMASS 10
#define LHIP 0.085
#define LTHI 0.2
#define LCAL 0.2

#define F_MAX 240
#define MU 0.6

const double ALPHA = 1e-10;
const double WEIGHT[13] = {0.5, 0.5, 50, 20, 20, 80, 0, 0, 0.2, 0.05, 0.05, 0.05, 0.f};
const double BODY_INERTIA[9] = {0.060288,0,0, 0,0.081321,0, 0,0,0.12107};

#endif //RAISIM_MPCDESCRIPTION_HPP
