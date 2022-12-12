//
// Created by hs on 22. 10. 14.
//

#include <canine_util/RobotMath.hpp>
#include <iostream>

#define PI 3.14159265359

Mat3<double> GetBaseRotationMat(const Vec4<double>& quat)
{
    Mat3<double> BaseRot;
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    BaseRot << 1-2*std::pow(y,2)-2*std::pow(z,2),                       2*x*y-2*w*z,                       2*x*z+2*w*y,
                                     2*x*y+2*w*z, 1-2*std::pow(x,2)-2*std::pow(z,2),                       2*y*z-2*w*x,
                                     2*x*z-2*w*y,                       2*y*z+2*w*x, 1-2*std::pow(x,2)-2*std::pow(y,2);
    return BaseRot;
}
Mat3<double> GetBaseRotationMatInverse(const Vec4<double>& quat)
{
    Mat3<double> BaseRot;
    BaseRot = GetBaseRotationMat(quat).inverse();
    return BaseRot;
}

Mat4<double> GetGlobal2BodyTransMat(const Vec4<double>& quat, const Vec3<double>& pos)
{
    Mat4<double> BaseRot;
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    BaseRot << 1-2*std::pow(y,2)-2*std::pow(z,2),                       2*x*y-2*w*z,                       2*x*z+2*w*y, pos[0],
                                     2*x*y+2*w*z, 1-2*std::pow(x,2)-2*std::pow(z,2),                       2*y*z-2*w*x, pos[1],
                                     2*x*z-2*w*y,                       2*y*z+2*w*x, 1-2*std::pow(x,2)-2*std::pow(y,2), pos[2],
                                               0,                                 0,                                 0, 1;
    return BaseRot;
}

Mat4<double> GetBody2GlobalTransMat(const Vec4<double>& quat, const Vec3<double>& pos)
{
    Mat4<double> BaseRot;
    BaseRot = GetGlobal2BodyTransMat(quat, pos).inverse();
    return BaseRot;
}

void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip,const double& thi,const double& cal)
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
    *Base2Foot = Bas2Hip*Hip2Thi*Thi2Cal*Cal2Foo;
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

void TransformQuat2Euler(const Vec4<double>& quat, double* euler)
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

Mat3<double> GetSkew(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.0, -r(2), r(1),
            r(2), 0.0, -r(0),
            -r(1), r(0), 0.0;
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

void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg)
{
    double alpha;
    double beta;

    if(leg == 0 || leg == 2) //Right Leg
    {
        alpha = acos(abs(footPos[1])/sqrt(pow(footPos[1],2)+pow(footPos[2],2)));
        beta = acos(LEN_HIP/sqrt(pow(footPos[1],2)+pow(footPos[2],2)));
        if (footPos[1] >= 0)
        {
            jointPos[0] = PI-beta-alpha;
        }
        else
        {
            jointPos[0] = alpha-beta;
        }
    }
    else  //Left Leg
    {
        alpha = acos(abs(footPos[1])/sqrt(pow(footPos[1],2)+pow(footPos[2],2)));
        beta = acos(LEN_HIP/sqrt(pow(footPos[1],2)+pow(footPos[2],2)));
        if (footPos[1] >= 0)
        {
            jointPos[0] = beta - alpha;
        }
        else
        {
            jointPos[0] = alpha+beta-PI;
        }
    }

    double zdot = -sqrt(pow(footPos[1],2)+pow(footPos[2],2)-pow(0.107496,2));
    double d = sqrt(pow(footPos[0],2)+pow(zdot,2));
    double phi = acos(abs(footPos[0])/ d);
    double psi = acos(pow(d,2)/(2*LEN_THI*d));

    if (footPos[0] < 0)
    {
        jointPos[1] = PI/2 - phi + psi;
    }
    else if(footPos[0] == 0)
    {
        jointPos[1] = psi;
    }
    else
    {
        jointPos[1] = phi + psi - PI/2;
    }
    jointPos[2] = -acos((pow(d,2)-2*pow(LEN_CAL,2)) / (2*LEN_CAL*LEN_CAL));
}