
#ifndef RAISIM_CANMOTORRL_HPP
#define RAISIM_CANMOTORRL_HPP

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <net/if.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "SharedMemory.hpp"
#include "RobotDescription.hpp"

class CanMotorRL
{
public:
    CanMotorRL(std::string canName);
    void CanFunction();

private:
    void canInit();
    void canSend(int motorIndex, const u_int8_t* data);
    void canRead();
    void readEncoder();
    void readMotorErrorStatus();
    void stopMotor();
    void turnOnMotor();
    void turnOffMotor();
    void setTorque();

private:
    double enc2rad;
    double torque2int[MOTOR_NUM_PER_CAN];
    std::string mCanName;
    struct can_frame mFrame;
    int mMotorId[MOTOR_NUM_PER_CAN];
    int mEncoder[MOTOR_NUM_PER_CAN];
    int mEncoderMultiturnNum[MOTOR_NUM_PER_CAN];
    int mEncoderTemp[MOTOR_NUM_PER_CAN];
    int mEncoderPast[MOTOR_NUM_PER_CAN];
    int mEncoderRaw[MOTOR_NUM_PER_CAN];
    int mEncoderOffset[MOTOR_NUM_PER_CAN];
    int mSock;
    int mGearRatio;
    int mMotorTemperature[MOTOR_NUM_PER_CAN];
    int mMotorErrorCode[MOTOR_NUM_PER_CAN];
    double mAxis[MOTOR_NUM_PER_CAN];
    double mAngularPositionOffset[MOTOR_NUM_PER_CAN];
    double mAngularPosition[MOTOR_NUM_PER_CAN];
    double mAngularVelocity[MOTOR_NUM_PER_CAN];
    double mCurrentTorque[MOTOR_NUM_PER_CAN];
    double mMotorVoltage[MOTOR_NUM_PER_CAN];
};


#endif //RAISIM_CANMOTORRL_HPP
