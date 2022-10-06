
#ifndef RAISIM_CANMOTORFORWARD_HPP
#define RAISIM_CANMOTORFORWARD_HPP


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

class CANMotorForward {
public:
    CANMotorForward(std::string canName)
    {
        mCanName = canName;
        enc2rad = 2.0 * 3.141592 / 65535;
        mSock = 0;
        mGearRatio = 9;
        mSendedCommand = 0;
        torque2int[LFHR_IDX] = 29.9043;
        torque2int[LFHP_IDX] = 29.9043;
        torque2int[LFKP_IDX] = 24.0385;
        torque2int[RFHR_IDX] = 29.9043;
        torque2int[RFHP_IDX] = 29.9043;
        torque2int[RFKP_IDX] = 24.0385;
//        torque2int[LBHR_IDX] = 29.9043;
//        torque2int[LBHP_IDX] = 29.9043;
//        torque2int[LBKP_IDX] = 24.0385;
//        torque2int[RBHR_IDX] = 29.9043;
//        torque2int[RBHP_IDX] = 29.9043;
//        torque2int[RBKP_IDX] = 24.0385;

        mMotorId[LFHR_IDX] = MOTOR_LFHR_ID;
        mMotorId[LFHP_IDX] = MOTOR_LFHP_ID;
        mMotorId[LFKP_IDX] = MOTOR_LFKP_ID;
        mMotorId[RFHR_IDX] = MOTOR_RFHR_ID;
        mMotorId[RFHP_IDX] = MOTOR_RFHP_ID;
        mMotorId[RFKP_IDX] = MOTOR_RFKP_ID;
//        mMotorId[LBHR_IDX] = MOTOR_LBHR_ID;
//        mMotorId[LBHP_IDX] = MOTOR_LBHP_ID;
//        mMotorId[LBKP_IDX] = MOTOR_LBKP_ID;
//        mMotorId[RBHR_IDX] = MOTOR_RBHR_ID;
//        mMotorId[RBHP_IDX] = MOTOR_RBHP_ID;
//        mMotorId[RBKP_IDX] = MOTOR_RBKP_ID;

        mAxis[LFHR_IDX] = 1.0;
        mAxis[LFHP_IDX] = 1.0;
        mAxis[LFKP_IDX] = 1.0;
        mAxis[RFHR_IDX] = 1.0;
        mAxis[RFHP_IDX] = -1.0;
        mAxis[RFKP_IDX] = -1.0;
//        mAxis[LBHR_IDX] = -1.0;
//        mAxis[LBHP_IDX] = 1.0;
//        mAxis[LBKP_IDX] = 1.0;
//        mAxis[RBHR_IDX] = -1.0;
//        mAxis[RBHP_IDX] = -1.0;
//        mAxis[RBKP_IDX] = -1.0;

        mAngularPositionOffset[LFHR_IDX] = LFHR_POS_OFFSET;
        mAngularPositionOffset[LFHP_IDX] = LFHP_POS_OFFSET;
        mAngularPositionOffset[LFKP_IDX] = LFKP_POS_OFFSET;
        mAngularPositionOffset[RFHR_IDX] = RFHR_POS_OFFSET;
        mAngularPositionOffset[RFHP_IDX] = RFHP_POS_OFFSET;
        mAngularPositionOffset[RFKP_IDX] = RFKP_POS_OFFSET;
//        mAngularPositionOffset[LBHR_IDX] = LBHR_POS_OFFSET;
//        mAngularPositionOffset[LBHP_IDX] = LBHP_POS_OFFSET;
//        mAngularPositionOffset[LBKP_IDX] = LBKP_POS_OFFSET;
//        mAngularPositionOffset[RBHR_IDX] = RBHR_POS_OFFSET;
//        mAngularPositionOffset[RBHP_IDX] = RBHP_POS_OFFSET;
//        mAngularPositionOffset[RBKP_IDX] = RBKP_POS_OFFSET;

        for(int index = 0; index < MOTOR_NUM_PER_CAN; index++)
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
    double torque2int[MOTOR_NUM_PER_CAN];

    void CanFunction();

private:
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
    int mSendedCommand;
    int mGearRatio;
    int mMotorTemperature[MOTOR_NUM_PER_CAN];
    int mMotorErrorCode[MOTOR_NUM_PER_CAN];
    double mAxis[MOTOR_NUM_PER_CAN];
    double mAngularPositionOffset[MOTOR_NUM_PER_CAN];
    double mAngularPosition[MOTOR_NUM_PER_CAN];
    double mAngularVelocity[MOTOR_NUM_PER_CAN];
    double mCurrentTorque[MOTOR_NUM_PER_CAN];
    double mMotorVoltage[MOTOR_NUM_PER_CAN];

public:
    void canInit();
    void canSend(int motorIndex, const u_int8_t *data);
    void canRead();
    void readEncoder();
    void readMotorErrorStatus();
    void stopMotor();
    void turnOnMotor();
    void turnOffMotor();

    void setTorque(double* desiredTorque);
    void setVelocity(double* desiredVelocity);
    void setPosition(double* desiredPosition);

    can_frame getFrame() { return mFrame; }
    int getSock() { return mSock; }
};


#endif //RAISIM_CANMOTORFORWARD_HPP
