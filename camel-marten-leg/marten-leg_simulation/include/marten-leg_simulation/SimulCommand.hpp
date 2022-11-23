//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULCOMMAND_HPP
#define RAISIM_SIMULCOMMAND_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <fstream>
#include <iostream>
#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>
#include <Eigen/Eigen>

class SimulCommand
{
public:
    SimulCommand();
    void commandFunction();

private:
    void writeToCSVfile();
    void initializeDS4();
    void readDS4();
    Eigen::MatrixXd positionTrackingData = Eigen::MatrixXd(2,5000);
    Eigen::MatrixXd GRFEstimatorData = Eigen::MatrixXd(4,5000);
    int ds4Fd;
    int ds4NumOfAxis;
    int ds4NumOfButton;
    char ds4Name[80];
    std::vector<char> ds4Button;
    std::vector<int> ds4Axis;
};

#endif //RAISIM_SIMULCOMMAND_HPP
