#ifndef RAISIM_MOTORCAN_H
#define RAISIM_MOTORCAN_H

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

/*
 * two RMD-X8-pro-V2 are used in the robot.
 */
class MotorCAN {
public:
    MotorCAN(std::string canName)
    {
        mCanName = canName;
        enc2rad = 2.0 * 3.141592 / 65535;
        torque2int = 24.0385;
        mSock = 0;
        mGearRatio = 9;
        mSendedCommand = 0;
        mMotorId[HIP_IDX] = MOTOR_HIP_ID;
        mMotorId[KNEE_IDX] = MOTOR_KNEE_ID;
        mAngularPositionOffset[HIP_IDX] = HIP_POS_OFFSET;
        mAngularPositionOffset[KNEE_IDX] = KNEE_POS_OFFSET;

        for(int index = 0; index < MOTOR_NUM; index++)
        {
            mEncoder[index] = 0;
            mEncoderMultiturnNum[index] = 0;
            mEncoderTemp[index] = 35000;
            mEncoderPast[index] = 35000;
            mEncoderRaw[index] = 0;
            mEncoderOffset[index] = 0;
            mMotorTemperature[index] = 0;
            mMotorErrorCode[index] = 0;
            mAngularPosition[index] = 0;
            mAngularVelocity[index] = 0;
            mCurrentTorque[index] = 0;
            mMotorVoltage[index] = 0;
        }
    }
    double enc2rad;
    double torque2int;

private:
    std::string mCanName;
    struct can_frame mFrame;
    int mMotorId[MOTOR_NUM];
    int mEncoder[MOTOR_NUM];
    int mEncoderMultiturnNum[MOTOR_NUM];
    int mEncoderTemp[MOTOR_NUM];
    int mEncoderPast[MOTOR_NUM];
    int mEncoderRaw[MOTOR_NUM];
    int mEncoderOffset[MOTOR_NUM];
    int mSock;
    int mSendedCommand;
    int mGearRatio;
    int mMotorTemperature[MOTOR_NUM];
    int mMotorErrorCode[MOTOR_NUM];
    double mAngularPositionOffset[MOTOR_NUM];
    double mAngularPosition[MOTOR_NUM];
    double mAngularVelocity[MOTOR_NUM];
    double mCurrentTorque[MOTOR_NUM];
    double mMotorVoltage[MOTOR_NUM];

public:
    void canInit();
    void canSend(int motorIndex, const u_int8_t *data);
    void canRead();
    void readEncoder();
    void readMotorErrorStatus();
    void stopMotor();
    void turnOnMotor();
    void turnOffMotor();

    void setTorque(double desiredTorque[MOTOR_NUM]);
    void setVelocity(double desiredVelocity[MOTOR_NUM]);
    void setPosition(double desiredPosition[MOTOR_NUM]);

    can_frame getFrame() { return mFrame; }
    int getSock() { return mSock; }
};


#endif //RAISIM_MOTORCAN_H
