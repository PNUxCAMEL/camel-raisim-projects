#include <iostream>
#include "Eigen/Dense"

Eigen::Quaternion<double> qCurrent;
Eigen::Quaternion<double> qGoal;
Eigen::Quaternion<double> q;

static const double rad2deg = 180.0 / 3.141592;
static const double deg2rad = 3.141592 / 180.0;

struct Euler
{
    //radian
    double roll;
    double pitch;
    double yaw;
};

Eigen::Quaterniond euler2quaternion(Eigen::Vector3d euler)
{
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return q;
}

int main()
{
    int n;
    Eigen::MatrixXd m = Eigen::MatrixXd::Random(5, 3);
    std::cout << "m =" << std::endl << m << std::endl;

    m.conservativeResize(m.rows(), m.cols() + 2);
    std::cout << "m =" << std::endl << m << std::endl;

    Eigen::MatrixXd m2 = Eigen::MatrixXd::Random(5, 2);
    std::cout << "m2 =" << std::endl << m2 << std::endl;

    m.rightCols(2) = m2;
    std::cout << "m =" << std::endl << m << std::endl;

    Eigen::VectorXd currentPosition = Eigen::VectorXd::Random(9);
    Eigen::MatrixXd functionValues = Eigen::MatrixXd::Random(9, 1);
    std::cout << "currentPosition" << std::endl << currentPosition << std::endl;
    std::cout << "functionValues" << std::endl << functionValues << std::endl;

    functionValues.rightCols(1) = currentPosition;
    std::cout << "n2 : " << n << std::endl;

    std::cout << "functionValues" << std::endl << functionValues << std::endl;

    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd::Random(9, 1);
    Eigen::VectorXd zeros = Eigen::VectorXd(currentPosition.size());
    zeros.setZero();

    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    std::cout << "mFunctionValue" << std::endl << mFunctionValue << std::endl;

    //euler to quaternion
    float roll = 1.5707, pitch = 0, yaw = 0.707;
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;

    //quaternion to euler
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw" << std::endl << euler << std::endl;

    //SLERP
    Eigen::Vector3d eulerCurrent = { 0.0, 0.0, 0.0 };
    Eigen::Quaterniond qCurrent = euler2quaternion(eulerCurrent);
    Eigen::Quaterniond qGoal = euler2quaternion({ 90.0 * deg2rad, 90.0 * deg2rad, 90.0 * deg2rad });
    Eigen::Quaterniond qSLERP;

    qSLERP = qCurrent.slerp(0.5, qGoal);

    Eigen::Vector3d eulerSLERP = qSLERP.toRotationMatrix().eulerAngles(0, 1, 2);

    std::cout << "qSLERP = " << std::endl << qSLERP.coeffs() << std::endl;
    std::cout << "eulerSLERP = " << std::endl << eulerSLERP << std::endl;
    std::cout << "90.0*deg2rad = " << 90.0 * deg2rad << std::endl;
}