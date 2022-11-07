//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULCOMMAND_HPP
#define RAISIM_SIMULCOMMAND_HPP

#include <fstream>
#include <iostream>
#include <canine-leg-left_util/SharedMemory.hpp>
#include <canine-leg-left_util/RobotDescription.hpp>
#include <Eigen/Eigen>

class SimulCommand
{
public:
    void commandFunction();

private:
    void writeToCSVfile();
    Eigen::MatrixXd positionTrackingData = Eigen::MatrixXd(2,2500);
    Eigen::MatrixXd GRFEstimatorData = Eigen::MatrixXd(3,2500);
};

#endif //RAISIM_SIMULCOMMAND_HPP
