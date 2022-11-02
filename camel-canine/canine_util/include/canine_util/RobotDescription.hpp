//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTDESCRIPTION_H
#define RAISIM_ROBOTDESCRIPTION_H

constexpr int MOTOR_NUM = 12;
constexpr int MOTOR_NUM_PER_CAN = 6;

constexpr double GRAVITY = -9.81;
constexpr double BODYMASS = 10.0;
constexpr double LEN_HIP = 0.085;
constexpr double LEN_THI = 0.23;
constexpr double LEN_CAL = 0.23;

//TODO: change data structure of motor ID to keymap
constexpr int MOTOR_RFHR_ID = 0x145;
constexpr int MOTOR_RFHP_ID = 0x146;
constexpr int MOTOR_RFKP_ID = 0x147;
constexpr int MOTOR_LFHR_ID = 0x141;
constexpr int MOTOR_LFHP_ID = 0x142;
constexpr int MOTOR_LFKP_ID = 0x143;
constexpr int MOTOR_RBHR_ID = 0x145;
constexpr int MOTOR_RBHP_ID = 0x146;
constexpr int MOTOR_RBKP_ID = 0x147;
constexpr int MOTOR_LBHR_ID = 0x141;
constexpr int MOTOR_LBHP_ID = 0x142;
constexpr int MOTOR_LBKP_ID = 0x143;

constexpr double RFHR_POS_OFFSET = -0.4673;
constexpr double RFHP_POS_OFFSET = -2.5707;
constexpr double RFKP_POS_OFFSET = 2.6501;
constexpr double LFHR_POS_OFFSET = -0.2307;
constexpr double LFHP_POS_OFFSET = 2.0436;
constexpr double LFKP_POS_OFFSET = -3.3460;
constexpr double RBHR_POS_OFFSET = -0.2249;
constexpr double RBHP_POS_OFFSET = -2.5697;
constexpr double RBKP_POS_OFFSET = 2.5906;
constexpr double LBHR_POS_OFFSET = -0.4596;
constexpr double LBHP_POS_OFFSET = 1.9225;
constexpr double LBKP_POS_OFFSET = -3.3375;

//constexpr double LUMPED_MASS = 0.0;

enum LEG_INDEX
{
    R_FRON = 0,
    L_FRON,
    R_BACK,
    L_BACK
};

enum MOTOR_INDEX
{
    RFHR_IDX = 0,
    RFHP_IDX,
    RFKP_IDX,
    LFHR_IDX,
    LFHP_IDX,
    LFKP_IDX,
    RBHR_IDX,
    RBHP_IDX,
    RBKP_IDX,
    LBHR_IDX,
    LBHP_IDX,
    LBKP_IDX,
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

enum CONTROL_STATE
{
    STATE_CONTROL_STOP,
    STATE_MOTOR_OFF,
    STATE_READY,
    STATE_HOME_READY,
    STATE_HOME_CONTROL,
    STATE_PD_READY,
    STATE_PD_CONTROL,
    STATE_MPC_REDAY,
    STATE_MPC_CONTROL,
    STATE_WBC_READY,
    STATE_WBC_CONTROL,
};

enum CAN_STATE
{
    CAN_NO_ACT,
    CAN_INIT,
    CAN_MOTOR_ON,
    CAN_MOTOR_OFF,
    CAN_SET_TORQUE,
    CAN_READ_ERROR
};

enum VISUAL_STATE
{
    STATE_VISUAL_STOP,
    STATE_OPEN_RAISIM,
    STATE_UPDATE_VISUAL
};

enum GAIT_TYPE
{
    STAND = 0,
    TROT,
    TEST
};


#endif //RAISIM_ROBOTDESCRIPTION_H
