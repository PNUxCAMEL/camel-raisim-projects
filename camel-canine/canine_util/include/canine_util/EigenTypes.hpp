//
// Created by hs on 22. 8. 10.
//

#ifndef RAISIM_EIGENTYPES_H
#define RAISIM_EIGENTYPES_H

#include <eigen3/Eigen/Dense>

template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec13 = typename Eigen::Matrix<T, 13, 1>;

template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

#endif //RAISIM_EIGENTYPES_H
