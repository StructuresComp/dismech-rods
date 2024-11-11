#ifndef HEADLESS_SIM_ENV_H
#define HEADLESS_SIM_ENV_H

#include "base_sim_env.h"

class HeadlessSimEnv : public BaseSimEnv
{
  public:
    HeadlessSimEnv(const shared_ptr<World>& m_world, const RenderParams& render_params,
                   const shared_ptr<BaseLogger>& logger);
    ~HeadlessSimEnv() override;

    void stepSimulation() override;
    void runSimulation() override;
};

#endif  // HEADLESS_SIM_ENV
