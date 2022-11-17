//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULCOMMAND_HPP
#define RAISIM_SIMULCOMMAND_HPP

#include <fstream>
#include <iostream>
#include <marten-leg_util/SharedMemory.hpp>
#include <marten-leg_util/RobotDescription.hpp>
#include <Eigen/Eigen>

class SimulCommand
{
public:
    void commandFunction();

private:
    void writeToCSVfile();
    Eigen::MatrixXd positionTrackingData = Eigen::MatrixXd(2,5000);
    Eigen::MatrixXd GRFEstimatorData = Eigen::MatrixXd(4,5000);
};

#endif //RAISIM_SIMULCOMMAND_HPP
