//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>
#include <iostream>

#include "EigenTypes.hpp"
#include "RobotDescription.hpp"

const Mat4<double> BaseRotationMat(const Vec4<double>& quat);
void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const Vec4<double>& quat, const double& hip,const double& thi,const double& cal);
void TransformQuat2Euler(const double* quat, double* euler);
void GetJacobian(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int side);
Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r);
int8_t NearZero(float a);
int8_t NearOne(float a);
void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg);

#endif //RAISIM_ROBOTMATH_HPP
