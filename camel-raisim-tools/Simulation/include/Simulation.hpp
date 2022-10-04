#ifndef RAISIM_SIMULATION_HPP
#define RAISIM_SIMULATION_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

class Simulation
{
public:
    Simulation(raisim::World* world, double dT);

    void SetGroundProperty(std::string groundProperty);

protected:
    raisim::Ground* mGround;
};

#endif //RAISIM_SIMULATION_HPP
