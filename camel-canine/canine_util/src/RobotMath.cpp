//
// Created by hs on 22. 10. 14.
//

#include <canine_util/RobotMath.hpp>


template <class T>

T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

void TransformQuat2Euler(const double* quat, double* euler)
{
    //edge case!
    float as = t_min(-2.*(quat[1]*quat[3]-quat[0]*quat[2]),.99999);
    euler[0] = atan2(2.f*(quat[2]*quat[3]+quat[0]*quat[1]),sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
    euler[1] = asin(as);
    euler[2] = atan2(2.f*(quat[1]*quat[2]+quat[0]*quat[3]),sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
}

void GetJacobian(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int side)
{
    double s1 = std::sin(pos[0]);
    double s2 = std::sin(pos[1]);

    double c1 = std::cos(pos[0]);
    double c2 = std::cos(pos[1]);

    double s32 = std::sin(pos[1]+pos[2]);
    double c32 = std::cos(pos[1]+pos[2]);

    //right leg side = 1 / left leg side = -1
    J << 0,
            LEN_THI*c2+LEN_CAL*c32,
            LEN_CAL*c32,

            (-1)*side*LEN_HIP*s1-LEN_THI*c1*c2-LEN_CAL*c1*c32,
            LEN_THI*s1*s2+LEN_CAL*s1*s32,
            LEN_CAL*s1*s32,

            side*LEN_HIP*c1-LEN_THI*s1*c2-LEN_CAL*s1*c32,
            -LEN_THI*c1*s2-LEN_CAL*c1*s32,
            -LEN_CAL*c1*s32;
}

Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return cm;
}

int8_t NearZero(float a)
{
    return (a < 0.01 && a > -.01);
}

int8_t NearOne(float a)
{
    return NearZero(a-1);
}