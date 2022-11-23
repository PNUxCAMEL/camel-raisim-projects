
#ifndef RAISIM_T265_HPP
#define RAISIM_T265_HPP
#include <iostream>
#include <iomanip>
#include <mutex>

#include <librealsense2/rs.hpp>

class T265 {
public:
    T265();
    rs2_quaternion GetT265quat()const;
    rs2_vector GetT265vel() const;
    rs2_vector GetT265pos() const;
    void SetConfig();
    void ParseData();

private:
    rs2_quaternion quaternion_exp(rs2_vector v);
    rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b);
    rs2_pose predict_pose(rs2_pose & pose, float dt_s);

    ///variables
private:
    std::string mSerial;
    rs2::pipeline mPipe;
    rs2::config mCfg;
    rs2_quaternion mT265quat;
    rs2_vector mT265translation;
    rs2_vector mT265vel;
    rs2_vector mT265acc;
    rs2_vector mT265angVel;
    rs2_vector mT265angAcc;
};
#endif //RAISIM_T265_HPP
