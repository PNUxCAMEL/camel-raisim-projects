//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>

#include "EigenTypes.hpp"
#include "RobotDescription.hpp"

void TransformationBody2Foot(Mat4<double>* Bas2Foo, LEG_INDEX legIndex, Vec4<double> quat,const double& hip,const double& thi,const double& cal);
void TransformQuat2Euler(const Vec4<double>& quat, double* euler);
void GetJacobian(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int side);
Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r);
int8_t NearZero(float a);
int8_t NearOne(float a);

#endif //RAISIM_ROBOTMATH_HPP
