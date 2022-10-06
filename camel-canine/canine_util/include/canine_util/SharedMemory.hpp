//
// Created by camel on 22. 9. 13.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

#include "RobotDescription.hpp"

#define CMD_dT              0.001
#define CONTROL_dT          0.005
#define CAN_dT              0.005
#define VISUAL_dT           0.01
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define PI                  3.141592
#define R2D                 57.2957802
#define D2R                 0.0174533


typedef struct _UI_COMMAND_
{
    int userCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, *pUI_COMMAND;

typedef struct _SHM_
{
    bool newCommand;
    bool can1Status;
    bool can2Status;
    bool motorStatus;
    int controlState;
    int visualState;
    int can1State;
    int can2State;
    int motorErrorStatus[MOTOR_NUM];
    int motorTemp[MOTOR_NUM];
    double localTime;
    double basePosition[3];
    double baseVelocity[3];
    double baseEulerPosition[3];
    double baseEulerVelocity[3];
    double motorPosition[MOTOR_NUM];
    double motorVelocity[MOTOR_NUM];
    double motorTorque[MOTOR_NUM];
    double motorDesiredTorque[MOTOR_NUM];
    double motorVoltage[MOTOR_NUM];
}SHM, *pSHM;

typedef struct _CUSTOM_DATA_ {
    double       customVariableDouble[MAX_CUSTOM_DATA];
    int         customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, *pCUSTOM_DATA;



#endif //RAISIM_SHAREDMEMORY_H
