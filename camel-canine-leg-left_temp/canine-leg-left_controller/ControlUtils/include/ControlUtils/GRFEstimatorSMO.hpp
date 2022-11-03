//
// Created by jh on 22. 11. 3.
//

#ifndef RAISIM_GRFESTIMATORSMO_HPP
#define RAISIM_GRFESTIMATORSMO_HPP

#include <Eigen/Eigen>
#include <canine-leg-left_util/SharedMemory.hpp>

class GRFEstimatorSMO
{
public:
    GRFEstimatorSMO();
    void Estimate();

private:
    Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mTorque = Eigen::VectorXd(2);
};


#endif //RAISIM_GRFESTIMATORSMO_HPP
