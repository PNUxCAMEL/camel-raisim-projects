//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_MPCDESCRIPTION_HPP
#define RAISIM_MPCDESCRIPTION_HPP

#include <iostream>
#include <Eigen/Eigen>


const double ALPHA = 1e-10;
const double WEIGHT[13] = {1, 1, 50,
                           20, 20, 80,
                           0, 0, 0.2,
                           0.05, 0.05, 0.05,
                           0.f};

#endif //RAISIM_MPCDESCRIPTION_HPP
