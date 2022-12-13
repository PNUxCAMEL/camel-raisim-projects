//
// Created by camel on 22. 9. 13.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

#include "RobotDescription.hpp"
#include "EigenTypes.hpp"

#define CMD_dT              0.002
#define HIGH_CONTROL_dT     0.02
#define LOW_CONTROL_dT      0.005
#define GAIT_PERIOD         0.5
#define CAN_dT              0.0025
#define VISUAL_dT           0.01
#define IMU_dT              0.0001
#define ESTIMATOR_dT        0.001
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define R2D                 57.2957802
#define D2R                 0.0174533
#define MPC_HORIZON         5


typedef struct _UI_COMMAND_
{
    int userCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, * pUI_COMMAND;

typedef struct _SHM_
{
    int* gaitTable;
    bool newCommand;
    bool can1Status;
    bool can2Status;
    bool motorStatus;
    bool motorBackState;
    bool motorForeState;
    int LowControlState;
    int HighControlState;
    int visualState;
    int gaitState;
    int can1State;
    int can2State;
    int motorErrorStatus[MOTOR_NUM];
    int motorTemp[MOTOR_NUM];
    double localTime;
    Vec3<double> basePosition;
    Vec3<double> baseVelocity;
    Vec3<double> baseDesiredPosition;
    Vec3<double> baseDesiredVelocity;
    Vec3<double> baseDesiredAcceleration;
    Vec3<double> legDesiredPosition;
    Vec3<double> legDesiredVelocity;
    Vec3<double> legDesiredAcceleration;
    Vec3<double> mpcTorque[4];
    Vec3<double> bodyFootPosition[4];
    Vec3<double> globalFootPosition[4];
    Vec4<double> baseQuartPosition;
    Vec3<double> desiredLinearVelocity;
    Vec3<double> desiredAngularVelocity;

    double baseAcceleration[3];
    double baseEulerPosition[3];
    double baseEulerVelocity[3];

    double motorPosition[MOTOR_NUM];
    double motorVelocity[MOTOR_NUM];
    double motorTorque[MOTOR_NUM];
    double motorDesiredTorque[MOTOR_NUM];
    double motorVoltage[MOTOR_NUM];
    int gaitIteration;

} SHM, * pSHM;

typedef struct _CUSTOM_DATA_
{
    double customVariableDouble[MAX_CUSTOM_DATA];
    int customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, * pCUSTOM_DATA;



#endif //RAISIM_SHAREDMEMORY_H
