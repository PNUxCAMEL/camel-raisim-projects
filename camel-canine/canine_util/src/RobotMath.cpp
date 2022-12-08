//
// Created by hs on 22. 10. 14.
//

#include <canine_util/RobotMath.hpp>
#include <iostream>

const Mat4<double> BaseRotationMat(const Vec4<double>& quat)
{
    Mat4<double> BaseRot;
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    BaseRot << 1-2*std::pow(y,2)-2*std::pow(z,2),                       2*x*y-2*w*z,                       2*x*z+2*w*y, 0,
                                     2*x*y+2*w*z, 1-2*std::pow(x,2)-2*std::pow(z,2),                       2*y*z-2*w*x, 0,
                                     2*x*z-2*w*y,                       2*y*z+2*w*x, 1-2*std::pow(x,2)-2*std::pow(y,2), 0,
                                               0,                                 0,                                 0, 1;
    return BaseRot;
}

void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const Vec4<double>& quat, const double& hip,const double& thi,const double& cal)
{
    Mat4<double> Bas2Hip;
    Mat4<double> Hip2Thi;
    Mat4<double> Thi2Cal;
    Mat4<double> Cal2Foo;
    double kee = -(thi+cal);

    switch (legIndex)
    {
        case(R_FRON):
        {
            Bas2Hip <<             1,             0,              0,     0.175,
                                   0, std::cos(hip), -std::sin(hip),    -0.055,
                                   0, std::sin(hip),  std::cos(hip),         0,
                                   0,             0,              0,         1;
            Hip2Thi << std::cos(thi),             0,  std::sin(thi),    0.0705,
                                   0,             1,              0, -0.030496,
                      -std::sin(thi),             0,  std::cos(thi),         0,
                                   0,             0,              0,         1;
            Thi2Cal << std::cos(cal),             0,  std::sin(cal),         0,
                                   0,             1,              0,    -0.077,
                      -std::sin(cal),             0,  std::cos(cal),     -0.23,
                                   0,             0,              0,         1;
            Cal2Foo << std::cos(kee),             0,  std::sin(kee),         0,
                                   0,             1,              0,         0,
                      -std::sin(kee),             0,  std::cos(kee),     -0.23,
                                   0,             0,              0,         1;
            break;
        }
        case(L_FRON):
        {
            Bas2Hip <<             1,             0,              0,     0.175,
                                   0, std::cos(hip), -std::sin(hip),     0.055,
                                   0, std::sin(hip),  std::cos(hip),         0,
                                   0,             0,              0,         1;
            Hip2Thi << std::cos(thi),             0,  std::sin(thi),    0.0705,
                                   0,             1,              0,  0.030496,
                      -std::sin(thi),             0,  std::cos(thi),         0,
                                   0,             0,              0,         1;
            Thi2Cal << std::cos(cal),             0,  std::sin(cal),         0,
                                   0,             1,              0,     0.077,
                      -std::sin(cal),             0,  std::cos(cal),     -0.23,
                                   0,             0,              0,         1;
            Cal2Foo << std::cos(kee),             0,  std::sin(kee),         0,
                                   0,             1,              0,         0,
                      -std::sin(kee),             0,  std::cos(kee),     -0.23,
                                   0,             0,              0,         1;
            break;
        }
        case(R_BACK):
        {
            Bas2Hip <<             1,             0,              0,    -0.175,
                                   0, std::cos(hip), -std::sin(hip),    -0.055,
                                   0, std::sin(hip),  std::cos(hip),         0,
                                   0,             0,              0,         1;
            Hip2Thi << std::cos(thi),             0,  std::sin(thi),   -0.0705,
                                   0,             1,              0, -0.030496,
                      -std::sin(thi),             0,  std::cos(thi),         0,
                                   0,             0,              0,         1;
            Thi2Cal << std::cos(cal),             0,  std::sin(cal),         0,
                                   0,             1,              0,    -0.077,
                      -std::sin(cal),             0,  std::cos(cal),     -0.23,
                                   0,             0,              0,         1;
            Cal2Foo << std::cos(kee),             0,  std::sin(kee),         0,
                                   0,             1,              0,         0,
                      -std::sin(kee),             0,  std::cos(kee),     -0.23,
                                   0,             0,              0,         1;
            break;
        }
        case(L_BACK):
        {
            Bas2Hip <<             1,             0,              0,    -0.175,
                                   0, std::cos(hip), -std::sin(hip),     0.055,
                                   0, std::sin(hip),  std::cos(hip),         0,
                                   0,             0,              0,         1;
            Hip2Thi << std::cos(thi),             0,  std::sin(thi),   -0.0705,
                                   0,             1,              0,  0.030496,
                      -std::sin(thi),             0,  std::cos(thi),         0,
                                   0,             0,              0,         1;
            Thi2Cal << std::cos(cal),             0,  std::sin(cal),         0,
                                   0,             1,              0,     0.077,
                      -std::sin(cal),             0,  std::cos(cal),     -0.23,
                                   0,             0,              0,         1;
            Cal2Foo << std::cos(kee),             0,  std::sin(kee),         0,
                                   0,             1,              0,         0,
                      -std::sin(kee),             0,  std::cos(kee),     -0.23,
                                   0,             0,              0,         1;

            break;
        }
        default:
        {
            break;
        }
    }
    *Base2Foot = BaseRotationMat(quat)*Bas2Hip*Hip2Thi*Thi2Cal*Cal2Foo;
}

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
