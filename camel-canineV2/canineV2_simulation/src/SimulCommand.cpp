//
// Created by hs on 22. 10. 28.
//

#include <canineV2_simulation/SimulCommand.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

void SimulCommand::commandFunction()
{
    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case CAN_ON:
            {
                std::cout << "=====Gait is changed=====" << std::endl;
                sharedMemory->gaitState = TROT;
                sharedMemory->gaitIteration = 0;
                break;
            }
            case VISUAL_ON:
            {
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->visualState = STATE_VISUAL_STOP;
                break;
            }
            case HOME:
            {
                sharedMemory->HighControlState = STATE_HOME_STAND_UP_READY;
                break;
            }
            case PD_CMD:
            {
                sharedMemory->HighControlState = STATE_HOME_STAND_DOWN_READY;
                break;
            }
            case CUSTOM_1:
            {
                sharedMemory->HighControlState = STATE_MPC_UP_REDAY;
                break;
            }
            case CUSTOM_2:
            {
                sharedMemory->HighControlState = STATE_MPC_DOWN_REDAY;
                break;
            }
            default:
                break;
        }
    }
}