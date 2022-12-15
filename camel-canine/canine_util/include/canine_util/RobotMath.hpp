//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>

#include "EigenTypes.hpp"
#include "RobotDescription.hpp"

Mat3<double> GetBaseRotationMat(const Vec4<double>& quat);
Mat3<double> GetBaseRotationMatInverse(const Vec4<double>& quat);
Mat4<double> GetGlobal2BodyTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
Mat4<double> GetBody2GlobalTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip,const double& thi,const double& cal);
void TransformQuat2Euler(const Vec4<double>& quat, double* euler);
void GetJacobian(Mat3<double>& J, const Vec3<double>& pos, const int& side);
Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r);
int8_t NearZero(float a);
int8_t NearOne(float a);
void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg);
void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double>& jointVel, Vec3<double>& jointAcc,
                         Vec3<double> footPos, const int& leg);
void GetBaseInverseKinematics(Vec3<double> pos[4], Vec3<double> vel[4], Vec3<double> acc[4],
                              const Vec3<double> foot[4],
                              const Vec3<double>& bodyPos,
                              const Vec3<double>& bodyRot);

#endif //RAISIM_ROBOTMATH_HPP