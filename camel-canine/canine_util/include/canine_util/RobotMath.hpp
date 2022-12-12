//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "EigenTypes.hpp"
#include "RobotDescription.hpp"

Mat3<double> GetBaseRotationMat(const Vec4<double>& quat);
Mat3<double> GetBaseRotationMatInverse(const Vec4<double>& quat);
Mat4<double> GetGlobal2BodyTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
Mat4<double> GetBody2GlobalTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip,const double& thi,const double& cal);
void TransformQuat2Euler(const Vec4<double>& quat, double* euler);
void GetJacobian(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int side);
Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r);
int8_t NearZero(float a);
int8_t NearOne(float a);
void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg);

#endif //RAISIM_ROBOTMATH_HPP
