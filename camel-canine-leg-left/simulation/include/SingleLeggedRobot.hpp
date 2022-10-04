//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMROBOT_H
#define RAISIM_SIMPLEPENDULUMROBOT_H

#include "Robot.hpp"

class SingleLeggedRobot : public Robot
{
public:
    SingleLeggedRobot(raisim::World* world, std::string urdfPath, std::string name)
        : Robot(world, urdfPath, name)
    {
        initialize();
    }

    void initialize() override;

private:
    double mMass = 0.0;             // should be update to real mass value
    double mLengthUpperLeg = 0.23;
    double mLengthLowerLeg = 0.23;
};


#endif //RAISIM_SIMPLEPENDULUMROBOT_H
