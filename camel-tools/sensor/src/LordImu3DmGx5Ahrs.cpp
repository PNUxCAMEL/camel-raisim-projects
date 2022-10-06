//
// Created by cha on 22. 10. 5.
//

#include "../include/LordImu3DmGx5Ahrs.hpp"


static constexpr uint32_t Hash(const char* c)
{
    return *c ? static_cast<uint32_t>(*c) + 33 * Hash(c + 1) : 5381;
};

LordImu3DmGx5Ahrs::LordImu3DmGx5Ahrs(mscl::InertialNode* node)
{
    mNode = node;
    std::cout << "Model Name : " << mNode->modelName() << std::endl;
    std::cout << "Model Number : " << mNode->modelNumber() << std::endl;
    std::cout << "Serial : " << mNode->serialNumber() << std::endl;
}

void LordImu3DmGx5Ahrs::SetConfig(int samplingHz)
{
    mscl::MipChannels ahrsImuChs;
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_STABILIZED_ACCEL_VEC, mscl::SampleRate::Hertz(100)));
    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);

    mscl::MipChannels estFilterChs;
    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);
}

void LordImu3DmGx5Ahrs::PareData()
{
    mscl::MipDataPackets packets = mNode->getDataPackets(500);
    for (mscl::MipDataPacket packet : packets)
    {
        mscl::MipDataPoints data = packet.data();
        mscl::MipDataPoint dataPoint;
        for (unsigned int itr = 0; itr < data.size(); itr++)
        {
            dataPoint = data[itr];
            uint32_t hash = Hash(dataPoint.channelName().c_str());
            switch (hash)
            {
            case Hash("roll"):
                mRoll = dataPoint.as_double();
                break;

            case Hash("pitch"):
                mPitch = dataPoint.as_double();
                break;

            case Hash("yaw"):
                mYaw = dataPoint.as_double();
                break;

            case Hash("orientQuaternion"):
                mOrientQuaternion = dataPoint.as_string();
                break;

            case Hash("scaledAccelX"):
                mScaledAccelX = dataPoint.as_double();
                break;
            case Hash("scaledAccelY"):
                mScaledAccelY = dataPoint.as_double();
                break;
            case Hash("scaledAccelZ"):
                mScaledAccelZ = dataPoint.as_double();
                break;

            case Hash("stabilizedAccelX"):
                mStabilizedAccelX = dataPoint.as_double();
                break;

            case Hash("stabilizedAccelY"):
                mStabilizedAccelY = dataPoint.as_double();
                break;

            case Hash("stabilizedAccelZ"):
                mStabilizedAccelZ = dataPoint.as_double();
                break;

            case Hash("scaledMagX"):
                mScaledMagX = dataPoint.as_double();
                break;
            case Hash("scaledMagY"):
                mScaledMagY = dataPoint.as_double();
                break;
            case Hash("scaledMagZ"):
                mScaledMagZ = dataPoint.as_double();
                break;

            case Hash("scaledGyroX"):
                mScaledGyroX = dataPoint.as_double();
                break;
            case Hash("scaledGyroY"):
                mScaledGyroY = dataPoint.as_double();
                break;
            case Hash("scaledGyroZ"):
                mScaledGyroZ = dataPoint.as_double();
                break;

            default:
                std::cout << "default" << std::endl;
                break;
            }
            if (!dataPoint.valid())
            {
                std::cout << "[Invalid] ";
            }
        }
    }
}

double* LordImu3DmGx5Ahrs::GetEulerAngle()
{
    mEulerAngle[0] = mRoll;
    mEulerAngle[1] = mPitch;
    mEulerAngle[2] = mYaw;

    return mEulerAngle;
}

double* LordImu3DmGx5Ahrs::GetQuaternion()
{
    std::string temp[4];
    for (int i = 1, j = 0; i < mOrientQuaternion.size() - 1; i++)
    {
        if (mOrientQuaternion[i] != ',')
        {
            temp[j].push_back(mOrientQuaternion[i]);
        }
        else
        {
            j++;
        }
    }
    mQuaternion[0] = std::stod(temp[0]);
    mQuaternion[1] = std::stod(temp[1]);
    mQuaternion[2] = std::stod(temp[2]);
    mQuaternion[3] = std::stod(temp[3]);

    return mQuaternion;
}

double* LordImu3DmGx5Ahrs::GetAccelVector()
{
    mScaledAccelVector[0] = mScaledAccelX;
    mScaledAccelVector[1] = mScaledAccelY;
    mScaledAccelVector[2] = mScaledAccelZ;

    return mScaledAccelVector;
}

double* LordImu3DmGx5Ahrs::GetStabilizedAccelVector()
{
    mStabilizedAccelVector[0] = mStabilizedAccelX;
    mStabilizedAccelVector[1] = mStabilizedAccelY;
    mStabilizedAccelVector[2] = mStabilizedAccelZ;

    return mStabilizedAccelVector;
}

double* LordImu3DmGx5Ahrs::GetMagVector()
{
    mScaledMagVector[0] = mScaledMagX;
    mScaledMagVector[1] = mScaledMagY;
    mScaledMagVector[2] = mScaledMagZ;

    return mScaledMagVector;
}

double* LordImu3DmGx5Ahrs::GetGyroVector()
{
    mScaledGyroVector[0] = mScaledGyroX;
    mScaledGyroVector[1] = mScaledGyroY;
    mScaledGyroVector[2] = mScaledGyroZ;

    return mScaledGyroVector;
}
