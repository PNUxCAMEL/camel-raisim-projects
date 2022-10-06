//
// Created by cha on 22. 10. 5.
//
#include "camel-tools/sensor.hpp"
#include "camel-tools/ThreadGenerator.hpp"

struct timespec TIME_START;
struct timespec TIME_END;

constexpr double R2D = 180.0 / 3.141592;

int timediff_us(struct timespec* before, struct timespec* after)
{
    return (after->tv_sec - before->tv_sec) * 1e6 + (after->tv_nsec - before->tv_nsec) / 1000;
}

int main(int argc, char** argv)
{
    const std::string mComPort = "/dev/ttyACM0";
    const mscl::Connection mConnection = mscl::Connection::Serial(mComPort);
    mscl::InertialNode node(mConnection);
    double* tempQuat;
    double* tempEuler;
    double* tempAcc;
    int iteration = 0;
    double totalTime = 0;
    try
    {
        LordImu3DmGx5Ahrs test(&node);
        test.SetConfig(1000);
        while (true)
        {
            usleep(100);
            iteration++;
            clock_gettime(CLOCK_REALTIME, &TIME_START);
            test.PareData();
            clock_gettime(CLOCK_REALTIME, &TIME_END);
            tempQuat = test.GetQuaternion();
            tempEuler = test.GetEulerAngle();
            tempAcc = test.GetAccelVector();
//            std::cout << "quaternion : " << tempQuat[0] << " " << tempQuat[1] << " " << tempQuat[2] << " " << tempQuat[3] << std::endl;

            if(tempEuler[0] > 0)
            {
                tempEuler[0] =  tempEuler[0] - 3.141592;
            }
            else
            {
                tempEuler[0] =  tempEuler[0] + 3.141592;
            }
            tempEuler[1] = -tempEuler[1];
            tempEuler[2] = -tempEuler[2];
            std::cout << " euler angle : " << tempEuler[0]*R2D << " " << tempEuler[1]*R2D << " " << tempEuler[2]*R2D << std::endl;
//            std::cout << " acc : " << tempAcc[0] << " " << tempAcc[1] << " " << tempAcc[2] << " " << std::endl;
            totalTime += timediff_us(&TIME_START, &TIME_END) * 0.001;
//            std::cout << "interval time per one parsing : " << timediff_us(&TIME_START, &TIME_END) * 0.001 << "ms" << std::endl;
            if (iteration == 10000)
            {
                std::cout << "total time : " << totalTime << std::endl;
                break;
            }

        }
    }
    catch (mscl::Error& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

    system("pause");
    return 0;
}

