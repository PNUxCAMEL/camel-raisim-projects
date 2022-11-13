//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_MPCDESCRIPTION_HPP
#define RAISIM_MPCDESCRIPTION_HPP

#include <iostream>
#include <Eigen/Eigen>


/* In real world
 * Horizon = 3 / Frequency = 100Hz
 * */
//const double ALPHA = 1e-6;
//const double WEIGHT[13] = {1, 1, 1,
//                           10, 10, 25,
//                           0.001, 0.001, 0.2,
//                           0.05, 0.05, 0.05,
//                           0.f};

/* In Simul
 * Horizon = 3 / Frequency = 100Hz
 * */
const double ALPHA = 1e-7;
const double WEIGHT[13] = {1, 1, 50,
                           30, 30, 80,
                           0, 0, 0.2,
                           1, 1, 1,
                           0.f};

#endif //RAISIM_MPCDESCRIPTION_HPP
