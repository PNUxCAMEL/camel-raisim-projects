#include <canine_util/Command.hpp>

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
                sharedMemory->can1State = CAN_INIT;
                sharedMemory->can2State = CAN_INIT;
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->simulState = ONLY_SIMULATION;
/*                sharedMemory->controlState = STATE_CONTROL_STOP;
                sharedMemory->can1State = CAN_MOTOR_ON;
                sharedMemory->can2State = CAN_MOTOR_ON;*/
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->simulState = WITH_SIMULATION;
/*                sharedMemory->controlState = STATE_CONTROL_STOP;
                sharedMemory->can1State = CAN_MOTOR_OFF;
                sharedMemory->can2State = CAN_MOTOR_OFF;*/
                break;
            }
            case HOME:
            {
                sharedMemory->controlState = STATE_HOME_READY;
                sharedMemory->can1State = CAN_SET_TORQUE;
                sharedMemory->can2State = CAN_SET_TORQUE;
                break;
            }
            case PD_CMD:
            {
                sharedMemory->controlState = STATE_PD_READY;
                sharedMemory->can1State = CAN_SET_TORQUE;
                sharedMemory->can2State = CAN_SET_TORQUE;

                break;
            }
            case CUSTOM_1:
            {
                sharedMemory->controlState = STATE_WBC_READY;
//                sharedMemory->can1State = CAN_SET_TORQUE;
//                sharedMemory->can2State = CAN_SET_TORQUE;
                break;
            }
            case CUSTOM_2:
            {
                sharedMemory->controlState = STATE_TROT_REDAY;
/*                sharedMemory->can1State = CAN_SET_TORQUE;
                sharedMemory->can2State = CAN_SET_TORQUE;*/
                break;
            }
            default:
                break;
        }
    }
}