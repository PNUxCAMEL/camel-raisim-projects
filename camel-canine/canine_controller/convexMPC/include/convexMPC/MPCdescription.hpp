//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_MPCDESCRIPTION_HPP
#define RAISIM_MPCDESCRIPTION_HPP

#include <iostream>
#include <Eigen/Eigen>


const double ALPHA = 1e-3;
const double WEIGHT[13] = {1, 1, 1,
                           1, 1, 50,
                           0, 0, 0.2,
                           0.05, 0.05, 0.05,
                           0.f};

#endif //RAISIM_MPCDESCRIPTION_HPP
