#ifndef RAISIM_CONTROLLER_HPP
#define RAISIM_CONTROLLER_HPP

#include "Robot.hpp"

class Controller
{
public:
    Controller(Robot* robot);

    virtual void DoControl() = 0;

protected:
    virtual void updateState() = 0;
    virtual void computeControlInput() = 0;
    virtual void setTrajectory() = 0;
    virtual void setControlInput() = 0;
    raisim::ArticulatedSystem* GetRaisimRobot() const;

    Robot* mRobot;
};


#endif //RAISIM_CONTROLLER_HPP
