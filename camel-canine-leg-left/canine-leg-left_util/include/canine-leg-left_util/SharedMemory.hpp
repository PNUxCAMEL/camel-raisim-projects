//
// Created by camel on 22. 9. 13.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

#include "RobotDescription.hpp"

#define CMD_dT              0.001
#define CONTROL_dT          0.001
#define VISUAL_dT           0.01
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define PI                  3.141592
#define R2D                 57.2957802
#define D2R                 0.0174533
#define GRAVITY             -9.81
#define BUF_SIZE            41

constexpr int       MPC_HORIZON     = 5;
constexpr double    MPC_dT          = 0.02;


typedef struct _UI_COMMAND_
{
    int userCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, * pUI_COMMAND;

typedef struct _SHM_
{
    bool newCommand;
    bool can1Status;
    bool can2Status;
    bool motorStatus;
    bool simulState;
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
    double baseQuartPosition[4];
    double baseEulerVelocity[3];
    double desiredHipVerticalPosition;
    double desiredHipVerticalVelocity;
    double hipVerticalPosition;
    double hipVerticalVelocity;
    double hipVerticalAcceleration;
    double motorPosition[MOTOR_NUM];
    double motorVelocity[MOTOR_NUM];
    double motorTorque[MOTOR_NUM];
    double motorDesiredTorque[MOTOR_NUM];
    double motorVoltage[MOTOR_NUM];

    double bufMotorPosition[MOTOR_NUM][BUF_SIZE];
    double bufMotorVelocity[MOTOR_NUM][BUF_SIZE];
    float GRFNetInputs[14];
    float estimatedGRFMLP;
    float estimatedGRFSMO;
    float estimatedGRFETO;
    float measuredGRF;
    float desiredGRF;

    double positionTrackingData[2][5000];
    double GRFEstimatorData[4][5000];
} SHM, * pSHM;

typedef struct _CUSTOM_DATA_
{
    double customVariableDouble[MAX_CUSTOM_DATA];
    int customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, * pCUSTOM_DATA;



#endif //RAISIM_SHAREDMEMORY_H
