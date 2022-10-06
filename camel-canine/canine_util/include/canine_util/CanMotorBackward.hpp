
#ifndef RAISIM_CANMOTORBACKWARD_HPP
#define RAISIM_CANMOTORBACKWARD_HPP

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

class CanMotorBackward {
public:
    CanMotorBackward(std::string canName)
    {
        mCanName = canName;
        enc2rad = 2.0 * 3.141592 / 65535;
        mSock = 0;
        mGearRatio = 9;
        mSendedCommand = 0;

        torque2int[LBHR_IDX - MOTOR_NUM_PER_CAN] = 29.9043;
        torque2int[LBHP_IDX - MOTOR_NUM_PER_CAN] = 29.9043;
        torque2int[LBKP_IDX - MOTOR_NUM_PER_CAN] = 24.0385;
        torque2int[RBHR_IDX - MOTOR_NUM_PER_CAN] = 29.9043;
        torque2int[RBHP_IDX - MOTOR_NUM_PER_CAN] = 29.9043;
        torque2int[RBKP_IDX - MOTOR_NUM_PER_CAN] = 24.0385;

        mMotorId[LBHR_IDX - MOTOR_NUM_PER_CAN] = MOTOR_LBHR_ID;
        mMotorId[LBHP_IDX - MOTOR_NUM_PER_CAN] = MOTOR_LBHP_ID;
        mMotorId[LBKP_IDX - MOTOR_NUM_PER_CAN] = MOTOR_LBKP_ID;
        mMotorId[RBHR_IDX - MOTOR_NUM_PER_CAN] = MOTOR_RBHR_ID;
        mMotorId[RBHP_IDX - MOTOR_NUM_PER_CAN] = MOTOR_RBHP_ID;
        mMotorId[RBKP_IDX - MOTOR_NUM_PER_CAN] = MOTOR_RBKP_ID;

        mAxis[LBHR_IDX - MOTOR_NUM_PER_CAN] = -1.0;
        mAxis[LBHP_IDX - MOTOR_NUM_PER_CAN] = 1.0;
        mAxis[LBKP_IDX - MOTOR_NUM_PER_CAN] = 1.0;
        mAxis[RBHR_IDX - MOTOR_NUM_PER_CAN] = -1.0;
        mAxis[RBHP_IDX - MOTOR_NUM_PER_CAN] = -1.0;
        mAxis[RBKP_IDX - MOTOR_NUM_PER_CAN] = -1.0;

        mAngularPositionOffset[LBHR_IDX - MOTOR_NUM_PER_CAN] = LBHR_POS_OFFSET;
        mAngularPositionOffset[LBHP_IDX - MOTOR_NUM_PER_CAN] = LBHP_POS_OFFSET;
        mAngularPositionOffset[LBKP_IDX - MOTOR_NUM_PER_CAN] = LBKP_POS_OFFSET;
        mAngularPositionOffset[RBHR_IDX - MOTOR_NUM_PER_CAN] = RBHR_POS_OFFSET;
        mAngularPositionOffset[RBHP_IDX - MOTOR_NUM_PER_CAN] = RBHP_POS_OFFSET;
        mAngularPositionOffset[RBKP_IDX - MOTOR_NUM_PER_CAN] = RBKP_POS_OFFSET;

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


#endif //RAISIM_CANMOTORBACKWARD_HPP
