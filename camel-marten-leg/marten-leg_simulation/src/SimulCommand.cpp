//
// Created by hs on 22. 10. 28.
//

#include <marten-leg_simulation/SimulCommand.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

void SimulCommand::commandFunction()
{
    if (sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch (incomingCommand)
        {
        case CAN_ON:
        {
            break;
        }
        case VISUAL_ON:
        {
            break;
        }
        case MOTOR_ON:
        {
            break;
        }
        case MOTOR_OFF:
        {
            break;
        }
        case HOME:
        {
            sharedMemory->controlState = STATE_HOME_STAND_UP_READY;
            break;
        }
        case PD_CMD:
        {
            sharedMemory->controlState = STATE_PD_READY;
            break;
        }
        case CUSTOM_1:
        {
            sharedMemory->controlState = STATE_TROT_REDAY;
            break;
        }
        case CUSTOM_2:
        {
            writeToCSVfile();
            break;
        }
        default:
            break;
        }
    }
}

void SimulCommand::writeToCSVfile()
{
//    for(int i = 0 ; i < 2 ; i++)
//    {
//        for(int j = 0 ; j < 2500 ; j++)
//        {
//            positionTrackingData(i,j) = sharedMemory->positionTrackingData[i][j];
//        }
//    }
//    std::string name1 = "positionTrackingData.csv";
//    std::ofstream file1(name1.c_str());
//    for(int  i = 0; i < positionTrackingData.rows(); i++)
//    {
//        for(int j = 0; j < positionTrackingData.cols(); j++)
//        {
//            std::string str = std::to_string(positionTrackingData(i,j));
//            if(j+1 == positionTrackingData.cols()){
//                file1<<str;
//            }else{
//                file1<<str<<',';
//            }
//        }
//        file1<<'\n';
//    }

    for(int i = 0 ; i < 4 ; i++)
    {
        for(int j = 0 ; j < 2500 ; j++)
        {
            GRFEstimatorData(i,j) = sharedMemory->GRFEstimatorData[i][j];
        }
    }

    std::string name2 = "GRFEstimatorData.csv";
    std::ofstream file2(name2.c_str());
    for(int  i = 0; i < GRFEstimatorData.rows(); i++)
    {
        for(int j = 0; j < GRFEstimatorData.cols(); j++)
        {
            std::string str = std::to_string(GRFEstimatorData(i,j));
            if(j+1 == GRFEstimatorData.cols()){
                file2<<str;
            }else{
                file2<<str<<',';
            }
        }
        file2<<'\n';
    }
    std::cout<<"Data is saved."<<std::endl;
}