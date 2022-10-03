#include <canine_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

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
                mCan->canInit();
                mCan->readMotorErrorStatus();
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->controlState = STATE_CONTROL_STOP;
                mCan->turnOnMotor();
                sharedMemory->controlState = STATE_READY;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->controlState = STATE_MOTOR_OFF;
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
                sharedMemory->controlState = STATE_READY;
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