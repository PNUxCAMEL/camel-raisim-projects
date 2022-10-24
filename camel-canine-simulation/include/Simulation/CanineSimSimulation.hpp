
#ifndef RAISIM_CANINESIMSIMULATION_HPP
#define RAISIM_CANINESIMSIMULATION_HPP

#include "Simulation.hpp"

namespace camel
{
    namespace raisim_tools
    {
        class CanineSimSimulation : public Simulation
        {
        public:
            CanineSimSimulation(raisim::World* world, double dT);
        };
    }
}
#endif //RAISIM_CANINESIMSIMULATION_HPP
