//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>

#include "EigenTypes.hpp"

void TransformQuat2Euler(const double* quat, double* euler);

#endif //RAISIM_ROBOTMATH_HPP
