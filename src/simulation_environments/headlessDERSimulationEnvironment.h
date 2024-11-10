#ifndef HEADLESS_DER_SIMULATION_ENVIRONMENT_H
#define HEADLESS_DER_SIMULATION_ENVIRONMENT_H

// Subclass of simulation environment
#include "derSimulationEnvironment.h"

class headlessDERSimulationEnvironment : public derSimulationEnvironment
{
  public:
    headlessDERSimulationEnvironment(const shared_ptr<world>& m_world,
                                     const renderParams& render_params,
                                     const shared_ptr<worldLogger>& logger);
    ~headlessDERSimulationEnvironment() override;

    void stepSimulation() override;
    void runSimulation() override;
};

#endif  // HEADLESS_DER_SIMULATION_ENVIRONMENT_H
