#include "implicit_midpoint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/external_forces/contact_force.h"
#include "rod_mechanics/force_container.h"

ImplicitMidpoint::ImplicitMidpoint(const std::shared_ptr<SoftRobots>& soft_robots,
                                   const std::shared_ptr<ForceContainer>& forces,
                                   const SimParams& sim_params, SolverType solver_type)
    : BackwardEuler(soft_robots, forces, sim_params, solver_type) {
}

ImplicitMidpoint::~ImplicitMidpoint() = default;

double ImplicitMidpoint::stepForwardInTime() {
    dt = orig_dt;

    // Newton Guess. Just use approximately last solution
    for (const auto& limb : limbs)
        limb->updateGuess(0.01, 0.5 * dt);

    // Perform collision detection if contact is enabled
    if (forces->cf)
        forces->cf->broadPhaseCollisionDetection();

    // Compute position at T=t+0.5dt
    dt = 2 * newtonMethod(0.5 * dt);

    for (const auto& limb : limbs) {
        // Compute velocity at T=t+0.5dt
        limb->u = (limb->x - limb->x0) / (0.5 * dt);

        // Compute position at T=t+dt
        limb->x = 2 * limb->x - limb->x0;
        limb->x0 = limb->x;

        // Compute velocity at T=t+dt
        limb->u = 2 * limb->u - limb->u0;
        limb->u0 = limb->u;
    }
    updateSystemForNextTimeStep();
    return dt;
}
