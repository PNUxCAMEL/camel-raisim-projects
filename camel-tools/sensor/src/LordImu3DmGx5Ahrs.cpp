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

    mEulerAngle[0] = 0;
    mEulerAngle[1] = 0;
    mEulerAngle[2] = 0;

    mQuaternion[0] = 0;
    mQuaternion[1] = 0;
    mQuaternion[2] = 0;
    mQuaternion[3] = 0;

    mScaledAccelVector[0] = 0;
    mScaledAccelVector[1] = 0;
    mScaledAccelVector[2] = 0;

    mStabilizedAccelVector[0] = 0;
    mStabilizedAccelVector[1] = 0;
    mStabilizedAccelVector[2] = 0;

    mScaledMagVector[0] = 0;
    mScaledMagVector[1] = 0;
    mScaledMagVector[2] = 0;

    mAngularVelocity[0] = 0;
    mAngularVelocity[1] = 0;
    mAngularVelocity[2] = 0;

    std::cout << "Model Name : " << mNode->modelName() << std::endl;
    std::cout << "Model Number : " << mNode->modelNumber() << std::endl;
    std::cout << "Serial : " << mNode->serialNumber() << std::endl;
}

void LordImu3DmGx5Ahrs::SetConfig(int samplingHz)
{
    mscl::MipChannels ahrsImuChs;
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_STABILIZED_ACCEL_VEC, mscl::SampleRate::Hertz(samplingHz)));
    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);

    mscl::MipChannels estFilterChs;
    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl::SampleRate::Hertz(samplingHz)));
    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl::SampleRate::Hertz(samplingHz)));
    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION, mscl::SampleRate::Hertz(samplingHz)));
    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);
}

void LordImu3DmGx5Ahrs::ParseData()
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
            case Hash("estRoll"):
                mEulerAngle[0] = dataPoint.as_double();
                break;
            case Hash("estPitch"):
                mEulerAngle[1] = dataPoint.as_double();
                break;
            case Hash("estYaw"):
                mEulerAngle[2] = dataPoint.as_double();
                break;

            case Hash("estOrientQuaternion"):
                mOrientQuaternion = dataPoint.as_string();
                break;

            case Hash("scaledAccelX"):
                mScaledAccelVector[0] = dataPoint.as_double();
                break;
            case Hash("scaledAccelY"):
                mScaledAccelVector[1] = dataPoint.as_double();
                break;
            case Hash("scaledAccelZ"):
                mScaledAccelVector[2] = dataPoint.as_double();
                break;

            case Hash("stabilizedAccelX"):
                mStabilizedAccelVector[0] = dataPoint.as_double();
                break;
            case Hash("stabilizedAccelY"):
                mStabilizedAccelVector[1] = dataPoint.as_double();
                break;
            case Hash("stabilizedAccelZ"):
                mStabilizedAccelVector[2] = dataPoint.as_double();
                break;

            case Hash("scaledMagX"):
                mScaledMagVector[0] = dataPoint.as_double();
                break;
            case Hash("scaledMagY"):
                mScaledMagVector[0] = dataPoint.as_double();
                break;
            case Hash("scaledMagZ"):
                mScaledMagVector[0] = dataPoint.as_double();
                break;

            case Hash("estAngularRateX"):
                mAngularVelocity[0] = dataPoint.as_double();
                break;
            case Hash("estAngularRateY"):
                mAngularVelocity[1] = dataPoint.as_double();
                break;
            case Hash("estAngularRateZ"):
                mAngularVelocity[2] = dataPoint.as_double();
                break;
            default:
//                std::cout << "default" << std::endl;
                break;
            }
            if (!dataPoint.valid())
            {
//                std::cout << "[Invalid] ";
            }
        }
    }
}

double* LordImu3DmGx5Ahrs::GetEulerAngle()
{
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
    return mScaledAccelVector;
}

double* LordImu3DmGx5Ahrs::GetStabilizedAccelVector()
{
    return mStabilizedAccelVector;
}

double* LordImu3DmGx5Ahrs::GetMagVector()
{
    return mScaledMagVector;
}

double* LordImu3DmGx5Ahrs::GetAngularVelocity()
{
    return mAngularVelocity;
}