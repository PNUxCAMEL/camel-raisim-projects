#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "RobotDescription.hpp"
#include "SharedMemory.hpp"

class Command
{
public:
    Command();

    void commandFunction();

private:
    void initializeDS4();
    void readDS4();
    int ds4Fd;
    int ds4NumOfAxis;
    int ds4NumOfButton;
    char ds4Name[80];
    std::vector<char> ds4Button;
    std::vector<int> ds4Axis;
};


#endif //RAISIM_COMMAND_H
