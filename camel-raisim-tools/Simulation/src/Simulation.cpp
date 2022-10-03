#include "Simulation.hpp"

Simulation::Simulation(raisim::World* world, double dT)
: mWorld(world)
{
    mWorld->setTimeStep(dT);
    mGround = mWorld->addGround(0, "gnd");
}

void Simulation::SetGroundProperty(std::string groundProperty)
{
    mGround->setAppearance(groundProperty);
}