#include <marten-leg_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

Command::Command()
{
//    initializeDS4();
}

void Command::commandFunction()
{
//    readDS4();
    if (sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch (incomingCommand)
        {
        case CAN_ON:
        {
            sharedMemory->canState = CAN_INIT;
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
            sharedMemory->canState = CAN_MOTOR_ON;
            break;
        }
        case MOTOR_OFF:
        {
            sharedMemory->controlState = STATE_CONTROL_STOP;
            sharedMemory->canState = CAN_MOTOR_OFF;
            break;
        }
        case HOME:
        {
            sharedMemory->controlState = STATE_HOME_STAND_UP_READY;
            sharedMemory->canState = CAN_SET_TORQUE;
            break;
        }
        case PD_CMD:
        {
            sharedMemory->controlState = STATE_PD_READY;
            sharedMemory->canState = CAN_SET_TORQUE;
            break;
        }
        case CUSTOM_1:
        {
            sharedMemory->controlState = STATE_MPC_READY;
            sharedMemory->canState = CAN_SET_TORQUE;
            break;
        }
        case CUSTOM_2:
        {
            sharedMemory->controlState = STATE_HOME_STAND_DOWN_READY;
            sharedMemory->canState = CAN_SET_TORQUE;
            break;
        }
        default:
            break;
        }
    }
}

void Command::initializeDS4()
{
    const char* ds4FilePath = "/dev/input/js0";
    ds4Fd = -1;
    ds4NumOfAxis = 0;
    ds4NumOfButton = 0;
    if ((ds4Fd = open(ds4FilePath, O_RDONLY)) < 0)
    {
        std::cerr << "Failed to open " << ds4FilePath << std::endl;
        return;
    }

    ioctl(ds4Fd, JSIOCGAXES, &ds4NumOfAxis);
    ioctl(ds4Fd, JSIOCGBUTTONS, &ds4NumOfButton);
    ioctl(ds4Fd, JSIOCGNAME(80), &ds4Name);

    ds4Button.resize(ds4NumOfButton, 0);
    ds4Axis.resize(ds4NumOfAxis, 0);

    std::cout << "Joystick: " << ds4Name << std::endl
        << "  axis: " << ds4NumOfAxis << std::endl
        << "  buttons: " << ds4NumOfButton << std::endl;

    fcntl(ds4Fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
}

void Command::readDS4()
{
    js_event js;

    read(ds4Fd, &js, sizeof(js_event));

    switch (js.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_AXIS:
//        if ((int)js.number >= ds4Axis.size())
//        {
//            std::cerr << "err:" << (int)js.number << std::endl;
//            continue;
//        }
        ds4Axis[(int)js.number] = js.value;
        break;
    case JS_EVENT_BUTTON:
//        if ((int)js.number >= ds4Button.size())
//        {
//            std::cerr << "err:" << (int)js.number << std::endl;
//            continue;
//        }
        ds4Button[(int)js.number] = js.value;
        break;
    }

    std::cout << "axis/10000: ";
    for (size_t i(0); i < ds4Axis.size(); ++i)
    {
        std::cout << " " << std::setw(2) << ds4Axis[i] / 10000;
    }
    std::cout << std::endl;

    std::cout << "  button: ";
    for (size_t i(0); i < ds4Button.size(); ++i)
    {
        std::cout << " " << (int)ds4Button[i];
    }
    std::cout << std::endl;

    usleep(100);
}