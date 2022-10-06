//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTDESCRIPTION_H
#define RAISIM_ROBOTDESCRIPTION_H

constexpr int MOTOR_NUM = 12;
//TODO: change data structure of motor ID to keymap
constexpr int MOTOR_LFHR_ID = 0x141;
constexpr int MOTOR_LFHP_ID = 0x142;
constexpr int MOTOR_LFKP_ID = 0x143;
constexpr int MOTOR_RFHR_ID = 0x145;
constexpr int MOTOR_RFHP_ID = 0x146;
constexpr int MOTOR_RFKP_ID = 0x147;
constexpr int MOTOR_LBHR_ID = 0x141;
constexpr int MOTOR_LBHP_ID = 0x142;
constexpr int MOTOR_LBKP_ID = 0x143;
constexpr int MOTOR_RBHR_ID = 0x145;
constexpr int MOTOR_RBHP_ID = 0x146;
constexpr int MOTOR_RBKP_ID = 0x147;

constexpr double LFHR_POS_OFFSET = -0.2307;
constexpr double LFHP_POS_OFFSET = 2.0436;
constexpr double LFKP_POS_OFFSET = -3.3460;
constexpr double RFHR_POS_OFFSET = -0.4673;
constexpr double RFHP_POS_OFFSET = -2.5707;
constexpr double RFKP_POS_OFFSET = 2.6501;
constexpr double LBHR_POS_OFFSET = -0.4596;
constexpr double LBHP_POS_OFFSET = 1.9225;
constexpr double LBKP_POS_OFFSET = -3.3375;
constexpr double RBHR_POS_OFFSET = -0.2249;
constexpr double RBHP_POS_OFFSET = -2.5697;
constexpr double RBKP_POS_OFFSET = 2.5906;
//constexpr double LUMPED_MASS = 0.0;

enum MOTOR_INDEX
{
    LFHR_IDX = 0,
    LFHP_IDX,
    LFKP_IDX,
    RFHR_IDX,
    RFHP_IDX,
    RFKP_IDX,
    LBHR_IDX,
    LBHP_IDX,
    LBKP_IDX,
    RBHR_IDX,
    RBHP_IDX,
    RBKP_IDX,
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

enum SIMULATION_STATE
{
    ONLY_SIMULATION,
    WITH_SIMULATION
};

#endif //RAISIM_ROBOTDESCRIPTION_H
