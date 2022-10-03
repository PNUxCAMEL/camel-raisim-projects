//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTDESCRIPTION_H
#define RAISIM_ROBOTDESCRIPTION_H

constexpr int MOTOR_NUM = 2;
//TODO: change data structure of motor ID to keymap
constexpr int MOTOR_HIP_ID = 0x143;
constexpr int MOTOR_KNEE_ID = 0x141;
constexpr double HIP_POS_OFFSET = 0.8300;
constexpr double KNEE_POS_OFFSET = -3.2021;
constexpr double LUMPED_MASS = 2.766;

enum MOTOR_INDEX
{
    HIP_IDX = 0,
    KNEE_IDX
};

enum CONTROL_STATE
{
    STATE_CONTROL_STOP,
    STATE_MOTOR_OFF,
    STATE_READY,
    STATE_HOME_READY,
    STATE_HOME_CONTROL,
    STATE_PD_READY,
    STATE_PD_CONTROL
};

enum VISUAL_STATE
{
    STATE_VISUAL_STOP,
    STATE_OPEN_RAISIM,
    STATE_UPDATE_VISUAL
};

enum COMMAND
{
    NO_ACT,
    CAN_ON,
    VISUAL_ON,
    MOTOR_ON,
    MOTOR_OFF,
    HOME,
    PD_CMD,
    CUSTOM_1,
    CUSTOM_2
};

#endif //RAISIM_ROBOTDESCRIPTION_H
