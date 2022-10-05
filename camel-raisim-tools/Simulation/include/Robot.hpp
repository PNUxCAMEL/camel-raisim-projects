#ifndef RAISIM_ROBOT_HPP
#define RAISIM_ROBOT_HPP

#include"raisim/World.hpp"

class Robot
{
public:
    Robot(raisim::World* world, std::string urdfPath, std::string name);

    virtual void initialize() = 0;

    void SetQ(Eigen::VectorXd Q) const;
    void SetTau(Eigen::VectorXd tau) const;
    int GetQDim() const;
    int GetQDDim() const;
    double GetWorldTime() const;
    Eigen::VectorXd GetQ() const;
    Eigen::VectorXd GetQD() const;
    Eigen::VectorXd GetCOM() const;
    Eigen::MatrixXd GetMassMatrix() const;
    std::string GetUrdfPath() const;
    raisim::ArticulatedSystem* GetRobot() const;

protected:
    raisim::World* mRobotWorld;
    raisim::ArticulatedSystem* mRobot;
    std::string mUrdfPath;

};


#endif //RAISIM_ROBOT_HPP