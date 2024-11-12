#include "world.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/external_forces/contact_force.h"
#include "rod_mechanics/external_forces/floor_contact_force.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/inner_forces/elastic_bending_force.h"
#include "rod_mechanics/inner_forces/elastic_stretching_force.h"
#include "rod_mechanics/inner_forces/elastic_twisting_force.h"
#include "rod_mechanics/inner_forces/inertial_force.h"
#include "rod_mechanics/soft_robots.h"
#include "time_steppers/backward_euler.h"
#include "time_steppers/forward_euler.h"
#include "time_steppers/implicit_midpoint.h"
#include "time_steppers/verlet_position.h"

World::World(const std::shared_ptr<SoftRobots>& soft_robots,
             const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params)
    : soft_robots(soft_robots), forces(forces), time_step(0), curr_time(0.0),
      total_time(sim_params.sim_time) {

    // Declare inner elastic forces. These should never be optional.
    forces->addForce(std::make_shared<ElasticStretchingForce>(soft_robots));
    forces->addForce(std::make_shared<ElasticBendingForce>(soft_robots));
    forces->addForce(std::make_shared<ElasticTwistingForce>(soft_robots));

    // Declare inertial force. Should be avoided for explicit methods
    if (sim_params.integrator != FORWARD_EULER && sim_params.integrator != VERLET_POSITION) {
        forces->addForce(std::make_shared<InertialForce>(soft_robots));
    }

    // Set up the time stepper
    switch (sim_params.integrator) {
        case FORWARD_EULER:
            stepper = std::make_shared<ForwardEuler>(soft_robots, forces, sim_params);
            break;
        case VERLET_POSITION:
            stepper = std::make_shared<VerletPosition>(soft_robots, forces, sim_params);
            break;
        case BACKWARD_EULER:
            stepper =
                std::make_shared<BackwardEuler>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            break;
        case IMPLICIT_MIDPOINT:
            stepper =
                std::make_shared<ImplicitMidpoint>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            break;
    }

    stepper->initStepper();

    if (sim_params.enable_2d_sim) {
        for (const auto& limb : soft_robots->limbs)
            limb->enable2DSim();
    }

    // Update boundary conditions
    updateCons();

    // Allocate every thing to prepare for the first iteration
    stepper->updateSystemForNextTimeStep();
}

World::~World() = default;

void World::updateCons() {
    for (const auto& limb : soft_robots->limbs)
        limb->updateMap();
    stepper->update();
}

void World::updateTimeStep() {
    curr_time += stepper->stepForwardInTime();
    time_step++;
}

void World::printSimData() {
    auto cf = forces->cf;
    auto ff = forces->ff;
    if (cf && ff) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f | "
                   "floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, cf->getNumCollisions(), cf->getMinDist(),
                   ff->num_contacts, ff->min_dist);
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s | "
                   "floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, 0, "N/A", ff->num_contacts, ff->min_dist);
        }
    }
    else if (cf) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f\n", curr_time, stepper->iter,
                   cf->getNumCollisions(), cf->getMinDist());
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s\n", curr_time, stepper->iter, 0,
                   "N/A");
        }
    }
    else if (ff) {
        printf("time: %.4f | iters: %i | floor_con: %i | f_min_dist: %.6f\n", curr_time,
               stepper->iter, ff->num_contacts, ff->min_dist);
    }
    else {
        printf("time: %.4f | iters: %i\n", curr_time, stepper->iter);
    }
}

bool World::simulationRunning() const {
    if (curr_time < total_time)
        return true;
    else {
        std::cout << "Completed simulation." << std::endl;
        return false;
    }
}

double World::getCoordinate(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->x[i];
}

VecX World::getM1(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->m1.row(i);
}

VecX World::getM2(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->m2.row(i);
}

int World::getTimeStep() const {
    return time_step;
}

double World::getCurrentTime() const {
    return curr_time;
}

bool World::floorExists() {
    return forces->ff != nullptr;
}

double World::getFloorZ() {
    if (forces->ff)
        return forces->ff->floor_z;
    throw std::runtime_error("Floor does not exist.");
}
