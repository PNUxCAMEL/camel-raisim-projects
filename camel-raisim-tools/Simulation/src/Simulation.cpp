#include "Simulation.hpp"

Simulation::Simulation(raisim::World* world, double dT)
{
    world->setTimeStep(dT);
    mGround = world->addGround(0, "gnd");
}

void Simulation::SetGroundProperty(std::string groundProperty)
{
    mGround->setAppearance(groundProperty);
}