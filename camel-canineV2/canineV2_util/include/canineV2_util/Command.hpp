#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"

class Command {
public:
    Command();

    void commandFunction();
};


#endif //RAISIM_COMMAND_H
