//
// Created by hs on 22. 10. 28.
//

#include <canine_simulation/SimulCommand.hpp>

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
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->visualState = STATE_VISUAL_STOP;
                break;
            }
            case MOTOR_OFF:
            {
                break;
            }
            case HOME:
            {
                sharedMemory->controlState = STATE_HOME_READY;
                break;
            }
            case PD_CMD:
            {
                sharedMemory->controlState = STATE_PD_READY;
                break;
            }
            case CUSTOM_1:
            {
                sharedMemory->controlState = STATE_WBC_READY;
                break;
            }
            case CUSTOM_2:
            {
                sharedMemory->controlState = STATE_MPC_REDAY;
                break;
            }
            default:
                break;
        }
    }
}