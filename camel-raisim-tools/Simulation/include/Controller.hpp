#ifndef RAISIM_CONTROLLER_HPP
#define RAISIM_CONTROLLER_HPP

#include "Robot.hpp"

class Controller
{
public:
    Controller(Robot* robot);

    Robot* GetRobot() const;
    virtual void DoControl() = 0;

private:
    virtual void updateState() = 0;
    virtual void computeControlInput() = 0;
    virtual void setTrajectory() = 0;
    virtual void setControlInput() = 0;
    Robot* mRobot;

};


#endif //RAISIM_CONTROLLER_HPP
