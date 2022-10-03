#include <canine_util/MotorCAN.hpp>

extern pSHM sharedMemory;

void MotorCAN::canInit()
{
    std::string command3 =
            "sudo ip link set " + mCanName + " up type can bitrate 1000000";
    const char *c3 = command3.c_str();
    system(c3);
    mSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSock == -1) {
        perror("Fail to create can socket for ");
        std::cout << mCanName << std::endl;
        return;
    }
    std::cout << "Success to create can socket for " << mCanName << std::endl;

    struct ifreq ifr;
    const char *canName = mCanName.c_str();
    strcpy(ifr.ifr_name, canName);
    int ret = ioctl(mSock, SIOCGIFINDEX, &ifr);
    if (ret == -1) {
        perror("Fail to get can interface index -");
        return;
    }
    std::cout << "Success to get can interface index: " << ifr.ifr_ifindex << std::endl;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(mSock, (struct sockaddr *) &addr, sizeof(addr));
    if (ret == -1) {
        perror("Fail to bind can socket -");
        return;
    }
    std::cout << "Success to bind can socket" << std::endl;
    sharedMemory->canStatus = true;
}

void MotorCAN::canSend(int motorIndex, const u_int8_t *data)
{
    u_int32_t tempid = mMotorId[motorIndex] & 0x1fffffff;
    mFrame.can_id = tempid;
    memcpy(mFrame.data, data, sizeof(data));
    mFrame.can_dlc = sizeof(data);
    int tx_bytes = write(mSock, &mFrame, sizeof(mFrame));
    if (tx_bytes == -1) {
        perror("Fail to transmit can");
        return;
    }
    mSendedCommand = mFrame.data[0];
}

void MotorCAN::canRead()
{
    read(mSock, &mFrame, sizeof(mFrame));
    int iteration = 0;
    while (mFrame.data[0] != mSendedCommand) {
        iteration++;
        read(mSock, &mFrame, sizeof(mFrame));
        if (iteration > 10000) {
            perror("Fail to receive can");
            break;
        }
    }
}

void MotorCAN::readEncoder()
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int8_t data[8] = {0X90, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        canSend(motorIndex, data);
        canRead();

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[2] + mFrame.data[3] * 256;
        mEncoderRaw[motorIndex] = mFrame.data[4] + mFrame.data[5] * 256;
        mEncoderOffset[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000))
        {
            mEncoderMultiturnNum[motorIndex] += 1;
        } else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000))
        {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;
        sharedMemory->motorPosition[motorIndex] = mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex];
    }
}

void MotorCAN::readMotorErrorStatus()
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int8_t data[8] = {0X9a, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        canSend(motorIndex, data);
        canRead();
        mMotorTemperature[motorIndex] = mFrame.data[1];
        mMotorVoltage[motorIndex] = (mFrame.data[4] + mFrame.data[5] * 256) * 0.1;
        mMotorErrorCode[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        sharedMemory->motorTemp[motorIndex] = mMotorTemperature[motorIndex];
        sharedMemory->motorVoltage[motorIndex] = mMotorVoltage[motorIndex];
        sharedMemory->motorErrorStatus[motorIndex] = mMotorErrorCode[motorIndex];
    }
}

void MotorCAN::turnOffMotor()
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int8_t data[8] = {0x80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        canSend(motorIndex, data);
        canRead();
    }
    sharedMemory->motorStatus = false;
}

void MotorCAN::stopMotor()
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int8_t data[8] = {0x81, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        canSend(motorIndex, data);
        canRead();
    }
}

void MotorCAN::turnOnMotor()
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int8_t data[8] = {0x88, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        canSend(motorIndex, data);
        canRead();
    }
    sleep(5);
    sharedMemory->motorStatus = true;
}

void MotorCAN::setTorque(double *desiredTorque)
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int16_t desiredCurrent = round(torque2int * desiredTorque[motorIndex]);
        u_int8_t currentLowerData;
        u_int8_t currentUpperData;

        currentLowerData = desiredCurrent % 256;
        desiredCurrent = desiredCurrent / 256;
        currentUpperData = desiredCurrent % 256;
        u_int8_t data[8] = {0Xa1, 0X00, 0X00, 0X00, currentLowerData, currentUpperData, 0X00, 0X00};
        canSend(motorIndex, data);

        canRead();
        int16_t currentTorque;
        int16_t angularVelocity;

        mMotorTemperature[motorIndex] = mFrame.data[1];
        currentTorque = mFrame.data[2] + mFrame.data[3] * 256;
        mCurrentTorque[motorIndex] = currentTorque / torque2int;
        angularVelocity = mFrame.data[4] + mFrame.data[5] * 256;
        mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio;

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000)) {
            mEncoderMultiturnNum[motorIndex] += 1;
        } else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000)) {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;

        sharedMemory->motorTemp[motorIndex] = mMotorTemperature[motorIndex];
        sharedMemory->motorDesiredTorque[motorIndex] = desiredTorque[motorIndex];
        sharedMemory->motorTorque[motorIndex] = mCurrentTorque[motorIndex];
        sharedMemory->motorVelocity[motorIndex] = mAngularVelocity[motorIndex];
        sharedMemory->motorPosition[motorIndex] = mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex];
    }
}

void MotorCAN::setVelocity(double *desiredVelocity)
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int32_t velocityInput = round(desiredVelocity[motorIndex] * R2D * 100 * mGearRatio);

        u_int8_t vel0;
        u_int8_t vel1;
        u_int8_t vel2;
        u_int8_t vel3;

        vel0 = velocityInput % 256;
        velocityInput = velocityInput / 256;
        vel1 = velocityInput % 256;
        velocityInput = velocityInput / 256;
        vel2 = velocityInput % 256;
        velocityInput = velocityInput / 256;
        vel3 = velocityInput % 256;

        u_int8_t data[8] = {0Xa2, 0X00, 0X00, 0X00, vel0, vel1, vel2, vel3};
        canSend(motorIndex, data);

        canRead();
        int16_t currentTorque;
        int16_t angularVelocity;

        mMotorTemperature[motorIndex] = mFrame.data[1];
        currentTorque = mFrame.data[2] + mFrame.data[3] * 256;
        mCurrentTorque[motorIndex] = currentTorque / torque2int;
        angularVelocity = mFrame.data[4] + mFrame.data[5] * 256;
        mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio;

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000)) {
            mEncoderMultiturnNum[motorIndex] += 1;
        } else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000)) {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;

        sharedMemory->motorTemp[motorIndex] = mMotorTemperature[motorIndex];
        sharedMemory->motorTorque[motorIndex] = mCurrentTorque[motorIndex];
        sharedMemory->motorVelocity[motorIndex] = mAngularVelocity[motorIndex];
        sharedMemory->motorPosition[motorIndex] = mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex];
    }
}

void MotorCAN::setPosition(double *desiredPosition)
{
    for(int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int32_t position_int = round(desiredPosition[motorIndex] * R2D * 100 * mGearRatio);

        u_int8_t pos0;
        u_int8_t pos1;
        u_int8_t pos2;
        u_int8_t pos3;

        u_int32_t temp = position_int;
        pos0 = temp % 256;
        temp = temp / 256;
        pos1 = temp % 256;
        temp = temp / 256;
        pos2 = temp % 256;
        temp = temp / 256;
        pos3 = temp % 256;

        u_int8_t data[8] = {0Xa3, 0X00, 0X00, 0X00, pos0, pos1, pos2, pos3};

        canSend(motorIndex, data);

        canRead();

        mMotorTemperature[motorIndex] = mFrame.data[1];

        int16_t currentTorque = mFrame.data[2] + mFrame.data[3] * 256;
        mCurrentTorque[motorIndex] = currentTorque / torque2int;

        int16_t angularVelocity = mFrame.data[4] + mFrame.data[5] * 256;
        mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio;

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000)) {
            mEncoderMultiturnNum[motorIndex] += 1;
        } else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000)) {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;

        sharedMemory->motorTemp[motorIndex] = mMotorTemperature[motorIndex];
        sharedMemory->motorTorque[motorIndex] = mCurrentTorque[motorIndex];
        sharedMemory->motorVelocity[motorIndex] = mAngularVelocity[motorIndex];
        sharedMemory->motorPosition[motorIndex] = mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex];
    }
}