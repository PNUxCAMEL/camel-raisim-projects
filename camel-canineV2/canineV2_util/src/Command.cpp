#include <canineV2_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

Command::Command()
{
}

void Command::commandFunction()
{
    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case CAN_ON:
            {
                sharedMemory->canFLState = CAN_INIT;
                sharedMemory->canFRState = CAN_INIT;
                sharedMemory->canRLState = CAN_INIT;
                sharedMemory->canRRState = CAN_INIT;
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->HighControlState = STATE_CONTROL_STOP;
                sharedMemory->canFLState = CAN_MOTOR_ON;
                sharedMemory->canFRState = CAN_MOTOR_ON;
                sharedMemory->canRLState = CAN_MOTOR_ON;
                sharedMemory->canRRState = CAN_MOTOR_ON;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->HighControlState = STATE_CONTROL_STOP;
                sharedMemory->canFLState = CAN_MOTOR_OFF;
                sharedMemory->canFRState = CAN_MOTOR_OFF;
                sharedMemory->canRLState = CAN_MOTOR_OFF;
                sharedMemory->canRRState = CAN_MOTOR_OFF;
                break;
            }
            case HOME:
            {
                sharedMemory->HighControlState = STATE_HOME_STAND_UP_READY;
                sharedMemory->canFLState = CAN_SET_TORQUE;
                sharedMemory->canFRState = CAN_SET_TORQUE;
                sharedMemory->canRLState = CAN_SET_TORQUE;
                sharedMemory->canRRState = CAN_SET_TORQUE;
                break;
            }
            case PD_CMD:
            {
                sharedMemory->HighControlState = STATE_HOME_STAND_DOWN_READY;
                sharedMemory->canFLState = CAN_SET_TORQUE;
                sharedMemory->canFRState = CAN_SET_TORQUE;
                sharedMemory->canRLState = CAN_SET_TORQUE;
                sharedMemory->canRRState = CAN_SET_TORQUE;
                break;
            }
            case CUSTOM_1:
            {
                sharedMemory->gaitState = TROT;
                sharedMemory->gaitIteration = 0;
                break;
            }
            case CUSTOM_2:
            {
                break;
            }
            default:
                break;
        }
    }
}