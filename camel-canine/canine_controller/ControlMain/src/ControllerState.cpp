//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/ControllerState.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

ControllerState::ControllerState(MotorCAN* can)
    : mIteration(0)
    , mCan(can)
{
}

void ControllerState::ControllerFunction()
{
    sharedMemory->localTime = mIteration * CONTROL_dT;
    mIteration++;
    switch(sharedMemory->controlState)
    {
        case STATE_CONTROL_STOP:
        {
            break;
        }
        case STATE_MOTOR_OFF:
        {
//            mCan->turnOffMotor();
            sharedMemory->controlState = STATE_CONTROL_STOP;
            break;
        }
        case STATE_READY:
        {
//            mCan->readMotorErrorStatus();
//            for(int index = 0; index < MOTOR_NUM; index++)
//            {
//                mTorque[index] = 0;
//            }
//            setControlInput();
            break;
        }
        case STATE_HOME_READY:
        {
            /*mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[HIP_IDX], 45.0 * D2R, sharedMemory->localTime, 1.0);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[KNEE_IDX], -90.0 * D2R, sharedMemory->localTime, 1.0);*/
//            double timeDuration = 1.0;
//            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[LFHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[LFHP_IDX], 0.881691, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[2].updateTrajectory(sharedMemory->motorPosition[LFKP_IDX], -1.11338, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[3].updateTrajectory(sharedMemory->motorPosition[RFHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[4].updateTrajectory(sharedMemory->motorPosition[RFHP_IDX], 0.881691, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[5].updateTrajectory(sharedMemory->motorPosition[RFKP_IDX], -1.11338, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[6].updateTrajectory(sharedMemory->motorPosition[LBHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[7].updateTrajectory(sharedMemory->motorPosition[LBHP_IDX], 0.881691, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[8].updateTrajectory(sharedMemory->motorPosition[LBKP_IDX], -1.11338, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[9].updateTrajectory(sharedMemory->motorPosition[RBHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[10].updateTrajectory(sharedMemory->motorPosition[RBHP_IDX], 0.881691, sharedMemory->localTime, timeDuration);
//            mCubicTrajectoryGen[11].updateTrajectory(sharedMemory->motorPosition[RBKP_IDX], -1.11338, sharedMemory->localTime, timeDuration);
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
//            mCan->readMotorErrorStatus();
//            doHomeControl();
            break;
        }
        case STATE_PD_READY:
        {
//            mBezierTrajectoryGen.updateTrajectory(sharedMemory->localTime, 1);
            sharedMemory->controlState = STATE_PD_CONTROL;
            break;
        }
        case STATE_PD_CONTROL:
        {
//            mCan->readMotorErrorStatus();
//            doPDControl();
            break;
        }
        default:
            break;
    }
}