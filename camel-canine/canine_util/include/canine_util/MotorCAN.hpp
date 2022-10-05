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
        mMotorId[LFHR_IDX] = MOTOR_LFHR_ID;
        mMotorId[LFHP_IDX] = MOTOR_LFHP_ID;
        mMotorId[LFKP_IDX] = MOTOR_LFKP_ID;
        mMotorId[RFHR_IDX] = MOTOR_RFHR_ID;
        mMotorId[RFHP_IDX] = MOTOR_RFHP_ID;
        mMotorId[RFKP_IDX] = MOTOR_RFKP_ID;
        mMotorId[LBHR_IDX] = MOTOR_LBHR_ID;
        mMotorId[LBHP_IDX] = MOTOR_LBHP_ID;
        mMotorId[LBKP_IDX] = MOTOR_LBKP_ID;
        mMotorId[RBHR_IDX] = MOTOR_RBHR_ID;
        mMotorId[RBHP_IDX] = MOTOR_RBHP_ID;
        mMotorId[RBKP_IDX] = MOTOR_RBKP_ID;

        mAxis[LFHR_IDX] = 1.0;
        mAxis[LFHP_IDX] = 1.0;
        mAxis[LFKP_IDX] = 1.0;
        mAxis[RFHR_IDX] = 1.0;
        mAxis[RFHP_IDX] = -1.0;
        mAxis[RFKP_IDX] = -1.0;
        mAxis[LBHR_IDX] = -1.0;
        mAxis[LBHP_IDX] = 1.0;
        mAxis[LBKP_IDX] = 1.0;
        mAxis[RBHR_IDX] = -1.0;
        mAxis[RBHP_IDX] = -1.0;
        mAxis[RBKP_IDX] = -1.0;

        mAngularPositionOffset[LFHR_IDX] = LFHR_POS_OFFSET;
        mAngularPositionOffset[LFHP_IDX] = LFHP_POS_OFFSET;
        mAngularPositionOffset[LFKP_IDX] = LFKP_POS_OFFSET;
        mAngularPositionOffset[RFHR_IDX] = RFHR_POS_OFFSET;
        mAngularPositionOffset[RFHP_IDX] = RFHP_POS_OFFSET;
        mAngularPositionOffset[RFKP_IDX] = RFKP_POS_OFFSET;
        mAngularPositionOffset[LBHR_IDX] = LBHR_POS_OFFSET;
        mAngularPositionOffset[LBHP_IDX] = LBHP_POS_OFFSET;
        mAngularPositionOffset[LBKP_IDX] = LBKP_POS_OFFSET;
        mAngularPositionOffset[RBHR_IDX] = RBHR_POS_OFFSET;
        mAngularPositionOffset[RBHP_IDX] = RBHP_POS_OFFSET;
        mAngularPositionOffset[RBKP_IDX] = RBKP_POS_OFFSET;
    }

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

private:
    double enc2rad;
    double torque2int;

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
    double mAxis[MOTOR_NUM];
    double mAngularPositionOffset[MOTOR_NUM];
    double mAngularPosition[MOTOR_NUM];
    double mAngularVelocity[MOTOR_NUM];
    double mCurrentTorque[MOTOR_NUM];
    double mMotorVoltage[MOTOR_NUM];

public:


    can_frame getFrame() { return mFrame; }
    int getSock() { return mSock; }
};


#endif //RAISIM_MOTORCAN_H
