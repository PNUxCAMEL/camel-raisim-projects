#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "MotorCAN.hpp"

class Command {
public:
    Command(MotorCAN* can);
    void commandFunction();

private:
    MotorCAN* mCan;
    void visualOn();
};


#endif //RAISIM_COMMAND_H
